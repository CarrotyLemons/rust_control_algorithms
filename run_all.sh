#!/bin/bash
cargo test
cd tests
uv run PID_visualiser.py
cargo clean