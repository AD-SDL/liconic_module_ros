FROM ghcr.io/ad-sdl/wei

LABEL org.opencontainers.image.source=https://github.com/AD-SDL/liconic_module
LABEL org.opencontainers.image.description="Drivers and REST API's for the liconic devices"
LABEL org.opencontainers.image.licenses=MIT

#########################################
# Module specific logic goes below here #
#########################################

RUN mkdir -p liconic_module

COPY ./liconic_driver liconic_module/liconic_driver
COPY ./scripts liconic_module/scripts
COPY ./README.md liconic_module/README.md
COPY ./pyproject.toml liconic_module/pyproject.toml
COPY ./tests liconic_module/tests

RUN --mount=type=cache,target=/root/.cache \
    pip install -e ./liconic_module

CMD ["python", "liconic_module/scripts/liconic_rest_node.py"]

#########################################
