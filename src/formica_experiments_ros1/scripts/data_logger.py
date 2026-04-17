"""
data_logger.py - ROS 1 Version
==============================
Shared CSV and console logging utility for all Chapter 6 experiments.

Usage:
    from formica_experiments.data_logger import CsvLogger, timestamped_filename

    logger = CsvLogger('exp1_calibration', ['sensor', 'measured', 'error'])
    logger.write_row(['LiDAR', 1.002, 0.002])
    logger.close()
"""

import csv
import os
import time
import datetime

DATA_DIR = os.path.expanduser('~/formica_experiments/data')


def ensure_data_dir():
    """Create the data directory if it does not already exist."""
    os.makedirs(DATA_DIR, exist_ok=True)
    return DATA_DIR


def timestamped_filename(experiment_name, extension='csv'):
    """Return a full file path with a UTC timestamp embedded in the filename."""
    ts = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d_%H%M%S')
    return os.path.join(ensure_data_dir(), '{0}_{1}.{2}'.format(experiment_name, ts, extension))


class CsvLogger:
    """Lightweight CSV writer for experiment data logging."""

    def __init__(self, experiment_name, headers=None):
        if headers is None:
            headers = []
        self.filepath = timestamped_filename(experiment_name)
        self._file = open(self.filepath, 'w')
        self._writer = csv.writer(self._file)
        self._writer.writerow(headers)
        self._file.flush()
        print('[DataLogger] Writing to {0}'.format(self.filepath))

    def write_row(self, row):
        """Append a single row to the CSV file."""
        self._writer.writerow(row)
        self._file.flush()

    def write(self, data):
        """Write a dictionary or single value."""
        if isinstance(data, dict):
            self._writer.writerow(data.values())
        else:
            self._writer.writerow([data])
        self._file.flush()

    def close(self):
        """Flush and close the file handle."""
        self._file.flush()
        self._file.close()
        print('[DataLogger] Closed {0}'.format(self.filepath))

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


class ExperimentSummary:
    """Accumulate numeric results and print a formatted summary table."""

    def __init__(self, experiment_label):
        self.label = experiment_label
        self.results = []

    def add(self, trial_id, metric, value, unit=''):
        self.results.append({
            'trial': trial_id,
            'metric': metric,
            'value': value,
            'unit': unit,
        })

    def print_summary(self):
        print('\n' + '=' * 60)
        print('  EXPERIMENT SUMMARY — {0}'.format(self.label))
        print('=' * 60)
        print('  {0:<8} {1:<30} {2:<12} {3}'.format('Trial', 'Metric', 'Value', 'Unit'))
        print('-' * 60)
        for r in self.results:
            print('  {0:<8} {1:<30} {2:<12.4f} {3}'.format(
                r['trial'], r['metric'], r['value'], r['unit']))
        print('=' * 60 + '\n')
