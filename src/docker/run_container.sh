chmod +x ./run.sh
sudo docker run -it \
-v "$(pwd)/../../":/workspace \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e XDG_RUNTIME_DIR=/tmp/runtime-root \
-e DISPLAY=unix$DISPLAY \
--net=host \
--gpus all \
--privileged \
--name mobinha_xingyou_0207_1 \
mobinha_xingyou1:240207_1


