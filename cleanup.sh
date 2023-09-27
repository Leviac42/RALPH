#!/bin/bash
podman container rm --all
podman image rm localhost/ros:latest
podman build -t ralph src/.devcontainer --rm