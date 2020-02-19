cd /tmp && \
git clone --recurse-submodules https://github.com/CIM-IEC/libcimpp.git && \
mkdir -p libcimpp/build && cd libcimpp/build && \
cmake -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 -DUSE_CIM_VERSION=IEC61970_16v29a .. && make -j$(nproc) install && \
rm -rf /tmp/libcimpp

cd /tmp && \
git clone --recurse-submodules https://git.rwth-aachen.de/acs/public/villas/node.git && \
mkdir -p villasnode/build && cd villasnode/build && \
cmake -DWITH_NODE_ETHERCAT=OFF -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 .. && make -j$(nproc) install && \
rm -rf /tmp/villasnode
