"""Asynchronous, transactional staging-episode writer."""

from __future__ import annotations

import json
import queue
import shutil
import threading
from pathlib import Path
from typing import Any

import cv2
import yaml


class EpisodeWriter:
    def __init__(
        self,
        output_root: Path,
        jpeg_quality: int,
        queue_size: int,
        enable_depth: bool = False,
    ) -> None:
        self.output_root = output_root
        self.jpeg_quality = int(jpeg_quality)
        self.queue: queue.Queue[Any] = queue.Queue(maxsize=int(queue_size))
        self.temp_dir: Path | None = None
        self.final_dir: Path | None = None
        self._thread: threading.Thread | None = None
        self._frames_file = None
        self._error: Exception | None = None
        self.frames_written = 0
        self.enable_depth = bool(enable_depth)

    @property
    def error(self) -> Exception | None:
        return self._error

    def start(self, episode_name: str, metadata: dict[str, Any]) -> Path:
        episodes = self.output_root / "staging" / "episodes"
        episodes.mkdir(parents=True, exist_ok=True)
        self.temp_dir = episodes / f".{episode_name}.incomplete"
        self.final_dir = episodes / episode_name
        if self.temp_dir.exists() or self.final_dir.exists():
            raise FileExistsError(f"episode already exists: {episode_name}")

        (self.temp_dir / "images" / "base").mkdir(parents=True)
        (self.temp_dir / "images" / "wrist").mkdir(parents=True)
        if self.enable_depth:
            (self.temp_dir / "depth" / "base").mkdir(parents=True)
            (self.temp_dir / "depth" / "wrist").mkdir(parents=True)
        self._write_yaml(self.temp_dir / "metadata.yaml", metadata)
        self._frames_file = (self.temp_dir / "frames.jsonl").open("w", encoding="utf-8")
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        return self.temp_dir

    def enqueue(
        self,
        record: dict[str, Any],
        base_bgr,
        wrist_bgr,
        base_depth=None,
        wrist_depth=None,
    ) -> None:
        if self._error:
            raise RuntimeError("episode writer failed") from self._error
        self.queue.put_nowait(
            (record, base_bgr, wrist_bgr, base_depth, wrist_depth)
        )

    def stop(self) -> None:
        if self._thread is None:
            return
        while self._thread.is_alive():
            try:
                self.queue.put(None, timeout=0.1)
                break
            except queue.Full:
                continue
        self._thread.join()
        self._thread = None
        if self._frames_file:
            self._frames_file.close()
            self._frames_file = None
        if self._error:
            raise RuntimeError("episode writer failed") from self._error

    def finalize(self, outcome: str, extra_metadata: dict[str, Any]) -> Path:
        if self.temp_dir is None or self.final_dir is None:
            raise RuntimeError("no episode to finalize")
        metadata_path = self.temp_dir / "metadata.yaml"
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
        metadata.update(extra_metadata)
        metadata["outcome"] = outcome
        metadata["frames"] = self.frames_written
        self._write_yaml(metadata_path, metadata)
        self.temp_dir.rename(self.final_dir)
        result = self.final_dir
        self.temp_dir = None
        self.final_dir = None
        return result

    def discard(self) -> None:
        if self.temp_dir and self.temp_dir.exists():
            shutil.rmtree(self.temp_dir)
        self.temp_dir = None
        self.final_dir = None

    def _worker(self) -> None:
        assert self.temp_dir is not None
        try:
            while True:
                item = self.queue.get()
                try:
                    if item is None:
                        return
                    record, base_bgr, wrist_bgr, base_depth, wrist_depth = item
                    frame_index = int(record["frame_index"])
                    base_rel = f"images/base/{frame_index:06d}.jpg"
                    wrist_rel = f"images/wrist/{frame_index:06d}.jpg"
                    self._write_jpeg(self.temp_dir / base_rel, base_bgr)
                    self._write_jpeg(self.temp_dir / wrist_rel, wrist_bgr)
                    record["observation.images.base"] = base_rel
                    record["observation.images.wrist"] = wrist_rel
                    if self.enable_depth:
                        if base_depth is None or wrist_depth is None:
                            raise RuntimeError("depth recording enabled but a depth frame is missing")
                        base_depth_rel = f"depth/base/{frame_index:06d}.png"
                        wrist_depth_rel = f"depth/wrist/{frame_index:06d}.png"
                        self._write_png(self.temp_dir / base_depth_rel, base_depth)
                        self._write_png(self.temp_dir / wrist_depth_rel, wrist_depth)
                        record["observation.depth.base"] = base_depth_rel
                        record["observation.depth.wrist"] = wrist_depth_rel
                    self._frames_file.write(json.dumps(record, separators=(",", ":")) + "\n")
                    self._frames_file.flush()
                    self.frames_written += 1
                finally:
                    self.queue.task_done()
        except Exception as exc:  # surfaced to the ROS node on the next tick/stop
            self._error = exc

    def _write_jpeg(self, path: Path, image) -> None:
        ok, encoded = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            raise RuntimeError(f"failed to encode {path.name}")
        path.write_bytes(encoded.tobytes())

    @staticmethod
    def _write_png(path: Path, image) -> None:
        ok, encoded = cv2.imencode(".png", image)
        if not ok:
            raise RuntimeError(f"failed to encode {path.name}")
        path.write_bytes(encoded.tobytes())

    @staticmethod
    def _write_yaml(path: Path, data: dict[str, Any]) -> None:
        path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
