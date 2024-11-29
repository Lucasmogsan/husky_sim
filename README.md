# Template for running ROS Noetic projects in docker container


# Using this for the first time:
1. Make a repository based on this template.
1. Add the submodules to packages folder (and remove placeholder)
1. Update the overlay and dev dependencied (apt and pip) to suit your project.
1. Update the `.env` file.
1. Make sure the entrypoint permissions are set: `chmod +x docker/entrypoint.sh`
1. Build the image (from this repositorys' root folder)
1. Start the container.


NB:
- In the overlay image the submodules (in packages folder) are copied over (from the Dockerfile)
- In the dev image the submodules (in packages folder) are a shared volume (from the docker-compose file)

# Files
- .devcontainer: vscode setup in container
- Docker: Dockerfile and entrypoint
- .env: Environment variables to be changed. Particularly important for ROS1 (ros master, nodes IP etc.)
- dependencies.repos: Repositories that are used, but not to be modified (e.g. drivers, tools etc)
- docker-compose: Compose file


# Docker (How to run docker environment)

Install docker and docker compose

Docker:
https://docs.docker.com/engine/install/

Docker compose:
https://docs.docker.com/compose/install/

Build the image:
```bash
docker compose build dev
```

Run the container:
```bash
docker compose up dev
```

Connect to the container:
```bash
docker exec -it $NAME bash
```

Remove everything, including stopped containers and all unused images (not just dangling ones):
```bash
docker system prune -a
```

# Submodules
Clone the repo with submodules:
```bash
git clone --recursive git@github.com:Lucasmogsan/ros1_template.git
```

Alternatively clone the repo and then get the submodules afterwards:

```bash
git clone git@github.com:Lucasmogsan/ros1_template.git
```

```bash
git submodule update --init --recursive
```


The main repo has references to the submodules. If these submodules are modified, then the main repo may need to update these references in order to pull the latest data.
```bash
git submodule update --remote
```

This modifies the references in the main repo, and these changes needs to be comitted and pushed.