.PHONY: build clean

build:
	docker exec -w /ros2_ws/rail-interface rail-runner bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'

clean:
	docker exec -w /ros2_ws/rail-interface rail-runner rm -rf build install log
