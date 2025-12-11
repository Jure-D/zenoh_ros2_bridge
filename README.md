# LAMPA Bridge


## Usage
```bash
podman run -it --network=host ghcr.io/jure-d/zenoh_ros2_bridge:latest
```

### Developemnt
#### Building
```bash
podman build -t zenoh_ros2_bridge .
```

#### Running
```bash
podman run -it --network=host zenoh_ros2_bridge:latest
```

#### Build and run
```bash
podman build -t zenoh_ros2_bridge . && podman run -it --network=host zenoh_ros2_bridge:latest
```