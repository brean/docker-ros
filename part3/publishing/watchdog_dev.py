import sys
import rclpy
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
        self._observer.schedule(
            event_handler, '/ws/publishing/', recursive=True)
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
