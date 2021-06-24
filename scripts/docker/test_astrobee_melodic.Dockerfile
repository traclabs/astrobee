# This will test an Astrobee melodic docker container.
# You must set the docker context to be the repository root directory

FROM astrobee/astrobee:latest-melodic

# Run tests
RUN cd /build/astrobee && make -j`nproc` tests && make -j`nproc` test
