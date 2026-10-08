#!/usr/bin/env python3
"""direct_cflib_coverage.py가 남긴 flight_log.csv를 읽어 오차 그래프를 그린다.
위: cross-track error(경로 이탈 거리) vs 시간, 아래: 고도 오차(z - HOVER_HEIGHT) vs 시간."""
import csv
import os
import sys
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

LOG_CSV = os.path.expanduser('~/natnet_ws/flight_log.csv')
HOVER_HEIGHT = 0.5
OUT_PNG = os.path.expanduser('~/natnet_ws/flight_error_plot.png')


def load_rows(log_path):
    if log_path.lower().endswith('.xlsx'):
        import openpyxl
        wb = openpyxl.load_workbook(log_path, data_only=True)
        ws = wb.worksheets[0]
        rows = list(ws.iter_rows(values_only=True))
        header = list(rows[0])
        return [dict(zip(header, row)) for row in rows[1:]]
    with open(log_path) as f:
        return list(csv.DictReader(f))


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else LOG_CSV
    t, cross_track, z_err = [], [], []
    for row in load_rows(csv_path):
        t.append(float(row['t']))
        cross_track.append(float(row['cross_track_err']))
        z_err.append(float(row['z']) - HOVER_HEIGHT)

    print(f'{len(t)}개 샘플 로드 ({csv_path})')
    print(f'cross-track error: 평균 {sum(cross_track)/len(cross_track):.3f}m, 최대 {max(cross_track):.3f}m')
    print(f'고도 오차: 평균 {sum(abs(e) for e in z_err)/len(z_err):.3f}m, 최대 {max(abs(e) for e in z_err):.3f}m')

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    ax1.plot(t, cross_track, 'b-')
    ax1.set_ylabel('cross-track error (m)')
    ax1.set_title('cross-track error vs time')
    ax1.set_ylim(0, 0.1)
    ax1.yaxis.set_major_locator(MultipleLocator(0.02))
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, z_err, 'r-')
    ax2.axhline(0, color='gray', linewidth=0.8)
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('altitude error (m)')
    ax2.set_title(f'altitude error (goal {HOVER_HEIGHT}m )')
    ax2.set_ylim(-0.02, 0.02)
    ax2.yaxis.set_major_locator(MultipleLocator(0.01))
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=150)
    print(f'그래프 저장됨: {OUT_PNG}')
    plt.show()


if __name__ == '__main__':
    main()
