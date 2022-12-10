#!/bin/bash

npm install -D --save autoprefixer
npm install -D --save postcss-cli

hugo --minify
hugo server --bind 0.0.0.0 -D