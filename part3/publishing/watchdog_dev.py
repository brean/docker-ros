import sys
import rclpy

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

from watchdog.observers.polling import PollingObserver
from watchdog.events import FileSystemEventHandler

from .node import MinimalPublisher


class CodeChangeHandler(FileSystemEventHandler):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def on_modified(self, event):
        # reload on modification
        self.node.trigger_restart()


class ReloadingPublisherMinimal(MinimalPublisher):
    def __init__(self):
        # create publisher and timer from original MinimalPublisher node
        super().__init__()
        self._running = True
        event_handler = CodeChangeHandler(self)
        self._observer = PollingObserver()

        watch_path_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='The directory path to watch for code changes.'
        )
        self.declare_parameter(
            'watch_path', '/ws/publishing/', watch_path_descriptor)
        self._watch_path = self.get_parameter(
            'watch_path').get_parameter_value().string_value

        self._observer.schedule(
            event_handler, self._watch_path, recursive=True)
        self.get_logger().info('Watchdog starting observer.')
        self._observer.start()

    def trigger_restart(self):
        self.get_logger().info('Watchdog triggered restart.')
        self.stop_observer()
        self._running = False
        # Exit the process. ROS launch will respawn it if configured.
        sys.exit(0)

    def spin(self):
        while rclpy.ok() and self._running:
            if self._observer.is_alive():
                self._observer.join(.5)
            rclpy.spin_once(self)

    def stop_observer(self):
        if self._observer and self._observer.is_alive():
            self._observer.stop()
            self.get_logger().info("Watchdog observer stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ReloadingPublisherMinimal()
        node.spin()
    except KeyboardInterrupt:
        if node:
            node.destroy_node()
            node.stop_observer()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
