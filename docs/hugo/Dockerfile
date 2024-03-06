FROM fedora:34

RUN dnf -y update

RUN dnf -y install \
  pandoc \
  wget \
  git \
  nodejs

WORKDIR /hugo
RUN wget https://github.com/gohugoio/hugo/releases/download/v0.95.0/hugo_extended_0.95.0_Linux-64bit.tar.gz && \
  tar -xvf hugo_extended_0.95.0_Linux-64bit.tar.gz && \
  install hugo /usr/bin

WORKDIR /website
# this is installed by the entrypoint script in the mounted volume
# npm install -D --save autoprefixer
# npm install -D --save postcss-cli

ENTRYPOINT ["./docker-entrypoint.sh"]

EXPOSE 1313
