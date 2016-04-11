#!/bin/sh

LIBPREFIX=$(realpath ${PWD}/../install)
CPPFLAGS="-I${LIBPREFIX}/include"
CFLAGS="-O3 -fPIC"
LDFLAGS="-L${LIBPREFIX}/lib -pthread"
LIBS="-ldl"

usage() {
cat << EOF
usage: $(basename $0) [mlx|verbs]

Compile the library and install it in the parent directory (${LIBPREFIX}).
To use it with DPDK you should set some environment variables:

export EXTRA_CFLAGS=-I${LIBPREFIX}/include
export EXTRA_LDFLAGS=-L${LIBPREFIX}/lib
export LD_LIBRARY_PATH=${LIBPREFIX}/lib
EOF
}

clean() {
	git clean -xffd
	git checkout .
}

libmlx_configure() {
	./autogen.sh
	./configure \
	--prefix=${LIBPREFIX} \
	--without-valgrind \
	--enable-shared \
	--enable-static \
        CPPFLAGS="${CPPFLAGS}" \
        CFLAGS="${CFLAGS}" \
	LDFLAGS="${LDFLAGS}" \
	LIBS="${LIBS}"
}

libibverbs_configure() {
	./autogen.sh
	./configure \
	--prefix=${LIBPREFIX} \
	--without-resolve-neigh \
	--enable-shared \
	--enable-static \
	--disable-examples \
	CFLAGS="${CFLAGS}"
}

while getopts h: o
do
        case "$o" in
        [?]|h)  usage
		exit 0
		;;
	*)
		usage
		exit 1
		;;
        esac
done

case "$1" in
	mlx)
		clean
		libmlx_configure
		make -j
		make install
		;;
	verbs)
		clean
		libibverbs_configure
		make -j
		make install
		;;
	*)
		usage
		exit 1
		;;
esac

echo ""
echo "You may need to set the following flags for DPDK"
echo "export EXTRA_CFLAGS=-I${LIBPREFIX}/include"
echo "export EXTRA_LDFLAGS=-L${LIBPREFIX}/lib"
echo "export LD_LIBRARY_PATH=${LIBPREFIX}/lib"
