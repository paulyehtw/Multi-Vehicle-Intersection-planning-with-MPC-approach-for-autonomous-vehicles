FROM gazebo:gzserver9

RUN mkdir /root/gazebo_data

COPY ./world/interview_city.world /root/gazebo_data/interview_city.world
COPY ./world/libworld_sim.so /root/gazebo_data/libworld_sim.so

# We need to set the plugin path AND be in the directory where the plugins are,
# otherwise Gazebo can not find the second shared object.
# Edit: both shared objects have been merged, but this is left here just in case.
ENV GAZEBO_PLUGIN_PATH /root/gazebo_data
WORKDIR /root/gazebo_data

CMD [ "gzserver", "/root/gazebo_data/interview_city.world" ]
