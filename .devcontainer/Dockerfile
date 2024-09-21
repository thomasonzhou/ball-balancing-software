FROM python:3.12

WORKDIR /app
COPY ./python /app

# RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
# --mount=target=/var/cache/apt,type=cache,sharing=locked \
# rm -f /etc/apt/apt.conf.d/docker-clean\
# && apt-get update && apt-get install -y python3-opencv python3-numpy

RUN pip3 install -e example_module
