# LAMPA Bridge


## Usage
### Building an image
```bash
podman build -t lampa_bridge .
```

### Running
```bash
podman run -it --network=host lampa_bridge:latest
```

### All at once
```bash
podman build -t lampa_bridge . && podman run -it --network=host lampa_bridge:latest
```