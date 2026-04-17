"""
data_logger.py
==============
Shared CSV and console logging utility for all Chapter 6 experiments.
Each experiment writes its own timestamped CSV file to ~/formica_experiments/data/.
"""

import csv
import os
import time
import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DATA_DIR = os.path.expanduser('~/formica_experiments/data')


def ensure_data_dir() -> str:
    """Create the data directory if it does not already exist and return its path."""
    os.makedirs(DATA_DIR, exist_ok=True)
    return DATA_DIR


def timestamped_filename(experiment_name: str, extension: str = 'csv') -> str:
    """Return a full file path with a UTC timestamp embedded in the filename."""
    ts = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d_%H%M%S')
    return os.path.join(ensure_data_dir(), f'{experiment_name}_{ts}.{extension}')


class CsvLogger:
    """
    Lightweight CSV writer that opens a file once and appends rows.
    Usage:
        logger = CsvLogger('exp1_calibration', ['sensor', 'measured', 'error'])
        logger.write_row(['LiDAR', 1.002, 0.002])
        logger.close()
    """

    def __init__(self, experiment_name: str, headers: list):
        self.filepath = timestamped_filename(experiment_name)
        self._file = open(self.filepath, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(headers)
        self._file.flush()
        print(f'[DataLogger] Writing to {self.filepath}')

    def write_row(self, row: list) -> None:
        """Append a single row to the CSV file."""
        self._writer.writerow(row)
        self._file.flush()

    def close(self) -> None:
        """Flush and close the file handle."""
        self._file.flush()
        self._file.close()
        print(f'[DataLogger] Closed {self.filepath}')

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


class ExperimentSummary:
    """
    Accumulate numeric results and print a formatted summary table at the end
    of each experiment trial sequence.
    """

    def __init__(self, experiment_label: str):
        self.label = experiment_label
        self.results: list = []

    def add(self, trial_id: int, metric: str, value: float, unit: str = '') -> None:
        self.results.append({
            'trial': trial_id,
            'metric': metric,
            'value': value,
            'unit': unit,
        })

    def print_summary(self) -> None:
        print('\n' + '=' * 60)
        print(f'  EXPERIMENT SUMMARY — {self.label}')
        print('=' * 60)
        print(f'  {"Trial":<8} {"Metric":<30} {"Value":<12} {"Unit"}')
        print('-' * 60)
        for r in self.results:
            print(f'  {r["trial"]:<8} {r["metric"]:<30} {r["value"]:<12.4f} {r["unit"]}')
        print('=' * 60 + '\n')


def main(args=None):
    """Standalone node that echoes logged messages from other experiment nodes."""
    rclpy.init(args=args)
    node = Node('data_logger_node')
    node.get_logger().info('DataLogger node active — monitoring /experiment_log topic.')

    def cb(msg: String):
        node.get_logger().info(f'[LOG] {msg.data}')

    node.create_subscription(String, '/experiment_log', cb, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
