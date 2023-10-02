#!/bin/bash
podman container rm --all
podman image rm localhost/ralph:latest
podman build -t ralph src/.devcontainer --rm
podman push localhost/ralph:latest tinkster42/ralph:latest