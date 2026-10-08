#!/usr/bin/env python3
"""coverage_guidance_sim_refined.py가 남긴 flight_log CSV를 읽어 오차 그래프를 그린다.
위: cross-track error(경로 이탈 거리) vs 시간, 아래: 고도 오차(z - HOVER_HEIGHT) vs 시간.
실비행용 plot_flight_error.py와 달리 y축을 데이터 범위에 맞춰 자동으로 잡는다 (시뮬 오차가 더 크게 나올 수 있어서)."""
import csv
import os
import sys
import matplotlib.pyplot as plt

HOVER_HEIGHT = 0.5


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
    csv_path = sys.argv[1]
    out_png = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(csv_path)[0] + '_error.png'

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
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, z_err, 'r-')
    ax2.axhline(0, color='gray', linewidth=0.8)
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('altitude error (m)')
    ax2.set_title(f'altitude error (goal {HOVER_HEIGHT}m)')
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    print(f'그래프 저장됨: {out_png}')


if __name__ == '__main__':
    main()
