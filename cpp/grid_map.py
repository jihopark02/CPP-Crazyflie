import json
import math
import os
import sys
from datetime import datetime

import pygame
from openpyxl import Workbook

from decomposition import decompose_trapezoidal, generate_palette
from cells import cells_from_labels
import tsp_cpp_ga
import path_smoothing
import astar

MAP_DIR = "map"
PATH_DIR = "path"

# 색상
COLOR_WINDOW_BG = (246, 247, 249)   # 그리드 바깥(여백/축 영역) 배경
COLOR_BG = (255, 255, 255)          # 그리드(작업 공간) 배경
COLOR_GRID_LINE = (226, 229, 234)
COLOR_GRID_BORDER = (110, 116, 128)
COLOR_OBSTACLE = (55, 60, 70)
COLOR_PANEL_BG = (236, 238, 242)
COLOR_PANEL_BORDER = (212, 216, 222)
COLOR_BUTTON = (52, 103, 168)
COLOR_BUTTON_HOVER = (72, 126, 196)
COLOR_BUTTON_ACTIVE = (46, 140, 92)
COLOR_BUTTON_TEXT = (255, 255, 255)
COLOR_TEXT = (40, 44, 52)
COLOR_LOAD_PANEL_BG = (250, 250, 252)
COLOR_LOAD_ITEM_HOVER = (218, 230, 245)
COLOR_PATH_LINE = (208, 36, 62)
COLOR_PATH_START = (22, 128, 60)
COLOR_START_MARKER = (37, 99, 235)
COLOR_START_FILL = (219, 232, 254)
COLOR_AXIS_TEXT = (85, 90, 100)
COLOR_AXIS_TICK = (110, 116, 128)

# UI 글꼴 (Windows: Segoe UI, 없으면 Arial 등으로 대체)
UI_FONT = "segoeui,arial,helvetica"

# 그리드 주변 여백 (픽셀): 왼쪽 = y축 눈금 숫자 + "y [m]", 아래 = x축 눈금
# 숫자 + "x [m]". 좌표계가 왼쪽 아래 원점이라 축도 왼쪽/아래에 그린다.
AXIS_MARGIN_LEFT = 70
AXIS_MARGIN_BOTTOM = 58
AXIS_MARGIN_TOP = 30
AXIS_MARGIN_RIGHT = 30
COLOR_AGENT = (245, 140, 20)
COLOR_AGENT_ALPHA = 110             # 에이전트 footprint 투명도 (0~255)
COLOR_AGENT_OUTLINE = (180, 95, 0)
COLOR_RAW_PATH = (150, 156, 168)
COLOR_UNROUNDED_CORNER = (170, 40, 190)
COLOR_CELL_ENTRY = (214, 48, 49)
COLOR_CELL_EXIT = (34, 139, 76)
COLOR_CELL_ORDER_BG = (40, 44, 52)
COLOR_CELL_ORDER_TEXT = (255, 255, 255)


class Button:
    def __init__(self, rect, text, font):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.font = font

    def draw(self, surface, active=False):
        mouse_pos = pygame.mouse.get_pos()
        hovered = self.rect.collidepoint(mouse_pos)
        if active:
            color = COLOR_BUTTON_ACTIVE
        else:
            color = COLOR_BUTTON_HOVER if hovered else COLOR_BUTTON
        pygame.draw.rect(surface, color, self.rect, border_radius=8)
        text_surf = self.font.render(self.text, True, COLOR_BUTTON_TEXT)
        text_rect = text_surf.get_rect(center=self.rect.center)
        surface.blit(text_surf, text_rect)

    def is_clicked(self, event):
        return (
            event.type == pygame.MOUSEBUTTONDOWN
            and event.button == 1
            and self.rect.collidepoint(event.pos)
        )


class GridMap:
    """
    사다리꼴 셀 분해 + TSP-CPP(통합 TSP + Coverage Path Planning) GA로 커버리지
    경로를 계획하는, 미터 단위 그리드 맵 에디터.

    셀 방문 순서와 각 셀의 진입/이탈점 선택을 tsp_cpp_ga.solve_tsp_cpp()가
    하나의 GA로 동시에 푼다 (Tung & Liu, "Solution of an Integrated Traveling
    Salesman and Coverage Path Planning Problem by Using a Genetic Algorithm
    with Modified Operators", IADIS IJCSIS 14(2), 2019). 각 셀은 8가지
    진입/이탈 조합("cell-path combination", 논문 Figure 2의 4가지 코너 조합을
    세로/가로 스윕 방향 2가지로 확장)을 가지며, GA의 염색체는 (셀 방문 순서, 각 셀의 조합 선택)을 동시에 인코딩해서
    둘을 함께 최적화한다. 단, 각 개체의 방문 순서가 정해지면 그 순서에 대한
    최적 진입/이탈 조합은 논문의 DP 서브루틴(Algorithm 1)으로 매 세대 다시
    구하므로, GA는 사실상 방문 순서만 탐색한다. 이후 (1) 셀의 진출점과 다음 셀의 진입점이
    바로 인접하지 않으면 astar.py로 장애물을 피하는 실제 경로를 이어붙이고,
    (2) path_smoothing.smooth_path()로 모든 모서리를 최소 회전반경
    (min_turn_radius_m) 이하의 원호로 둥글려 곡률 제약을 만족하는 부드러운
    경로를 만든다 (Höffmann, Patel & Büskens, "Optimal Coverage Path Planning
    for Agricultural Vehicles with Curvature Constraints", Agriculture 2023의
    스무딩 목표를 원호 필렛으로 구현 -- 자세한 내용은 path_smoothing.py 참고).

    - 그리드의 한 칸(subcell)은 로봇/에이전트 크기(subcell_size_m)와 같은
      D x D 정사각형이다. rows x cols 는 실제 작업 공간(가로/세로, 미터)을
      subcell_size_m 로 나눈 값으로, main.py에서 계산해 넘겨준다.
    - 화면 렌더링은 subcell_px(픽셀/subcell)로 축척을 고정하고, 왼쪽/아래쪽에
      미터 단위 눈금(축)을 그려서 실제 크기를 그대로 확인할 수 있게 한다.
      좌표계는 왼쪽 아래가 (0, 0)이 되도록 y축 눈금을 아래에서 위로 표시한다
      (내부 obstacles 배열은 기존처럼 row=0이 위쪽이라, 화면/좌표 표기만
      뒤집어서 보여준다).
    - 좌클릭(드래그 포함)으로 셀에 장애물을 생성/제거.
    - 하단 패널의 Decompose / Clear / Save / Load / Coverage Path 버튼:
        - Decompose: 현재 장애물 배치에 대해 사다리꼴(Boustrophedon) 셀 분해를
          수행하고, 각 셀을 서로 다른 색으로 칠해서 "셀 간" 구조를 보여준다.
        - Clear: 장애물을 모두 지우고 분해/커버리지 결과도 초기화.
        - Save / Load: 장애물 배치를 map/ 폴더에 JSON으로 저장/불러오기.
        - Coverage Path: 위에서 설명한 TSP-CPP GA -> A* 이어붙이기 -> 곡률
          스무딩을 실행하고, 각 셀의 진입(빨강)/이탈(초록) 지점과 방문 순서
          번호를 표시한 뒤, 에이전트가 부드러운 경로를 실제로 따라 움직이는
          애니메이션으로 보여준다.
        - Save Path: Coverage Path로 계산된 스무딩(곡률 제약 반영) 경로를
          path_point_spacing_m(main.py의 PATH_POINT_SPACING_M)
          간격으로 리샘플링해서 path/ 폴더에 Excel(.xlsx)로 저장한다.
          Coverage Path를 먼저 실행해야 한다.
    - 장애물을 편집하면(분해된 상태였다면) 분해 결과와 커버리지 경로가
      초기화된다. 맵이 바뀌면 다시 Decompose를 눌러야 하는데, 이는 논문
      제목의 "reconfigurable grid-map"을 그대로 반영한 것.
    """

    def __init__(
        self,
        rows,
        cols,
        subcell_size_m,
        subcell_px=100,
        panel_height=120,
        min_window_width=1000,
        fps=60,
        start_pos=(0, 0),
        anim_speed_m_per_sec=0.3,
        ga_params=None,
        min_turn_radius_m=0.35,
        arc_samples_per_corner=12,
        path_point_spacing_m=0.1,
    ):
        self.rows = rows
        self.cols = cols
        self.subcell_size_m = subcell_size_m
        self.subcell_px = subcell_px
        self.panel_height = panel_height
        self.fps = fps
        self.start_pos = tuple(start_pos)
        self.anim_speed = anim_speed_m_per_sec
        self.ga_params = dict(ga_params) if ga_params else {}
        self.min_turn_radius_m = min_turn_radius_m
        self.arc_samples_per_corner = arc_samples_per_corner
        # Save Path 버튼으로 내보낼 때, 스무딩된 경로를 따라 점을 몇 미터
        # 간격으로 찍을지. main.py에서 속성으로 조절한다.
        self.path_point_spacing_m = path_point_spacing_m

        self.grid_width = self.cols * self.subcell_px
        self.grid_height = self.rows * self.subcell_px

        # obstacles[row][col] = True/False  (row, col은 subcell 인덱스)
        self.obstacles = [[False for _ in range(self.cols)] for _ in range(self.rows)]

        # 셀 분해 결과
        self.labels = None
        self.num_cells = 0
        self.palette = []
        self.show_decomposition = False

        # 커버리지 경로 (TSP-CPP GA 풀이),
        # 곡률 제약을 반영해 스무딩한 경로 (path_smoothing), 그리고
        # 재생 애니메이션 상태.
        self.coverage_path = None  # A*로 이어붙인 그리드 경로: list[(row, col)]
        self.cell_markers = None   # 셀별 방문 순서 + 진입/이탈 지점
        self.smooth_path = None    # path_smoothing.SmoothPath
        self.coverage_stats = None
        self.show_coverage_path = False
        self.anim_progress = 0.0  # smooth_path를 따라 이동한 거리 (미터)
        self.animating = False

        pygame.init()
        pygame.display.set_caption(
            f"GA Coverage Path Planning - {self.cols * self.subcell_size_m:.2f}m x "
            f"{self.rows * self.subcell_size_m:.2f}m, D={self.subcell_size_m:.2f}m"
        )
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(UI_FONT, 18)
        self.small_font = pygame.font.SysFont(UI_FONT, 16)
        self.tiny_font = pygame.font.SysFont(UI_FONT, 13)
        self.axis_title_font = pygame.font.SysFont(UI_FONT, 15, bold=True)

        self.running = True

        # 드래그로 장애물 칠하기 상태
        self.dragging = False
        self.drag_paint_value = None
        self.last_drag_cell = None

        # Panel buttons: 버튼 너비는 실제 렌더링된 텍스트 폭 기준으로 계산해서
        # "Coverage Path"처럼 긴 라벨도 밖으로 튀어나오지 않게 한다.
        btn_h = 38
        gap = 10
        padding_x = 18

        button_labels = ["Decompose", "Clear", "Save", "Load", "Coverage Path", "Save Path"]
        button_widths = [self.font.size(label)[0] + padding_x * 2 for label in button_labels]
        buttons_row_w = sum(button_widths) + gap * (len(button_widths) - 1)

        # 창 크기: 맵이 작아져도 하단 패널(버튼 줄 + 상태 메시지)이 잘리지
        # 않도록, (그리드 + 축 여백) 폭 / 버튼 줄 폭 / min_window_width 중
        # 가장 큰 값으로 창 너비를 잡고, 그리드는 가로 가운데에 놓는다.
        plot_w = AXIS_MARGIN_LEFT + self.grid_width + AXIS_MARGIN_RIGHT
        self.screen_width = max(plot_w, buttons_row_w + 40, min_window_width)
        plot_h = AXIS_MARGIN_TOP + self.grid_height + AXIS_MARGIN_BOTTOM
        self.panel_top = plot_h
        self.screen_height = plot_h + self.panel_height

        # 그리드 왼쪽 위 모서리의 화면 좌표 (모든 그리드 좌표 변환의 기준점)
        self.origin_x = (self.screen_width - plot_w) // 2 + AXIS_MARGIN_LEFT
        self.origin_y = AXIS_MARGIN_TOP

        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))

        buttons = []
        x = (self.screen_width - buttons_row_w) // 2
        btn_y = self.panel_top + 14
        for label, btn_w in zip(button_labels, button_widths):
            buttons.append(Button((x, btn_y, btn_w, btn_h), label, self.font))
            x += btn_w + gap

        (
            self.decompose_button,
            self.clear_button,
            self.save_button,
            self.load_button,
            self.coverage_button,
            self.save_path_button,
        ) = buttons

        self.status_message = ""
        self.status_message_timer = 0

        # Load 창(오버레이) 상태
        self.load_overlay_open = False
        self.load_overlay_files = []
        self.load_overlay_rects = []

        os.makedirs(MAP_DIR, exist_ok=True)
        os.makedirs(PATH_DIR, exist_ok=True)

    # ---------- 그리드 보조 함수 ----------

    def pixel_to_cell(self, pos):
        x, y = pos
        x -= self.origin_x
        y -= self.origin_y
        if x < 0 or y < 0 or y >= self.grid_height:
            return None
        col = x // self.subcell_px
        row = y // self.subcell_px
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return row, col
        return None

    def pixel_to_meters(self, pos):
        """화면 픽셀 좌표를 왼쪽 아래가 원점인 (x, y) 미터 좌표로 변환한다."""
        x, y = pos
        x -= self.origin_x
        y -= self.origin_y
        mx = x / self.subcell_px * self.subcell_size_m
        my_from_top = y / self.subcell_px * self.subcell_size_m
        my = self.rows * self.subcell_size_m - my_from_top
        return mx, my

    def set_obstacle(self, row, col, value):
        if self.obstacles[row][col] == value:
            return
        self.obstacles[row][col] = value
        self._invalidate_decomposition()

    def paint_line(self, from_cell, to_cell, value):
        """from_cell과 to_cell 사이 칸들을 value로 채운다 (빠른 드래그 시 칸 누락 방지)."""
        r0, c0 = from_cell
        r1, c1 = to_cell
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        steps = max(dr, dc)
        if steps == 0:
            self.set_obstacle(r1, c1, value)
            return
        for i in range(steps + 1):
            r = round(r0 + (r1 - r0) * i / steps)
            c = round(c0 + (c1 - c0) * i / steps)
            self.set_obstacle(r, c, value)

    def clear_obstacles(self):
        self.obstacles = [[False for _ in range(self.cols)] for _ in range(self.rows)]
        self._invalidate_decomposition()
        self._set_status("Cleared obstacles")

    # ---------- 셀 분해 ----------

    def run_decomposition(self):
        self.labels, self.num_cells = decompose_trapezoidal(self.obstacles, self.rows, self.cols)
        self.palette = generate_palette(self.num_cells)
        self.show_decomposition = True
        self._set_status(f"Decomposed into {self.num_cells} cells")

    def _invalidate_decomposition(self):
        if self.show_decomposition:
            self.show_decomposition = False
            self.labels = None
        self._invalidate_coverage_path()

    # ---------- 커버리지 경로 (TSP-CPP GA -> A* 이어붙이기 -> 스무딩) ----------

    def compute_coverage_path(self):
        if self.obstacles[self.start_pos[0]][self.start_pos[1]]:
            self._set_status(f"Start {self.start_pos} is blocked by an obstacle")
            return

        if not self.show_decomposition or self.labels is None:
            self.run_decomposition()

        cells = cells_from_labels(self.labels, self.num_cells)
        if not cells:
            self._set_status("No free cells to cover")
            return

        # subcell이 맵 안에 있고 장애물이 아닌지 검사하는 함수 -- GA의
        # 장애물 인식 전이 비용 계산과 아래 A* 이어붙이기에서 함께 쓴다.
        def is_free(r, c):
            if not (0 <= r < self.rows and 0 <= c < self.cols):
                return False
            return not self.obstacles[r][c]

        # (1) 셀 방문 순서와 (2) 각 셀의 진입/이탈 조합을 GA 하나로 동시에
        # 푼다 (tsp_cpp_ga.py / TSP-CPP, Tung & Liu 2019, WJTung/GA-TSPCPP
        # 참고). 각 셀 내부 경로는 고정된 boustrophedon(지그재그) 스윕
        # (8가지 조합 중 하나)이라서 셀별 TSP나 탐욕적 진입점 탐색이 따로
        # 필요 없고, GA가 조합을 직접 고른다 (모든 개체에 논문의 DP
        # 서브루틴을 적용). GA에 그리드를 넘겨주므로, 셀의 이탈점과 다음
        # 셀의 진입점이 상하좌우로 바로 붙어 있지 않으면 실제 A* 경로
        # 길이로 전이 비용을 계산한다 -- 아래 이어붙이기 단계가 만드는
        # 경로와 똑같은 거리다.
        order, combo, cell_paths, ga_stats = tsp_cpp_ga.solve_tsp_cpp(
            cells, self.start_pos, is_free=is_free, rows=self.rows, cols=self.cols, **self.ga_params
        )

        raw_path = []
        cell_markers = []
        for visit_idx, cell_idx in enumerate(order):
            k = combo[cell_idx]
            raw_path.extend(cell_paths[cell_idx].paths[k])
            cell_markers.append({
                "order": visit_idx + 1,
                "cell_id": cells[cell_idx].id,
                "entry": cell_paths[cell_idx].entries[k],
                "exit": cell_paths[cell_idx].exits[k],
            })

        # 연속한 두 경로 점이 상하좌우로 붙어 있지 않으면 (거의 항상 한
        # 셀의 이탈점 -> 다음 셀의 진입점), 장애물을 뚫고 지나갈 수 있는
        # 직선 점프 대신 장애물을 피하는 실제 경로로 그 사이를 채운다.
        # GA가 투어 비용을 계산할 때 쓴 것과 같은 A*를 재사용한다.
        stitched_path, detours = astar.stitch_path(is_free, self.rows, self.cols, raw_path)

        # 이어붙인 그리드 경로를 실제 미터 좌표로 바꿔서 곡률 제약
        # 스무딩(path_smoothing.py)을 적용한다.
        path_m = [self._subcell_to_meters(r, c) for r, c in stitched_path]
        smooth = path_smoothing.smooth_path(
            path_m, self.min_turn_radius_m, samples_per_arc=self.arc_samples_per_corner
        )

        self.coverage_path = stitched_path
        self.cell_markers = cell_markers
        self.smooth_path = smooth
        rounded = [c for c in smooth.corners if not c.unrounded]
        unrounded = [c for c in smooth.corners if c.unrounded]
        limited = [c for c in rounded if c.limited_by_segment_length]
        self.coverage_stats = {
            "path_length": len(stitched_path),
            "num_cells": len(order),
            "tsp_cpp_length": ga_stats["best_length"],
            "generations_run": ga_stats["generations_run"],
            "smooth_length_m": smooth.total_length,
            "num_corners": len(smooth.corners),
            "num_limited_corners": len(limited),
            "num_unrounded_corners": len(unrounded),
            "num_astar_detours": len(detours),
        }
        self.show_coverage_path = True
        self.anim_progress = 0.0
        self.animating = smooth.total_length > 0

        limited_note = f", {len(limited)} tight-radius" if limited else ""
        unrounded_note = f", {len(unrounded)} unrounded (too sharp)" if unrounded else ""
        astar_note = f", {len(detours)} A* detours" if detours else ""
        self._set_status(
            f"TSP-CPP GA ({ga_stats['generations_run']} gens): {len(order)} cells, "
            f"tour={ga_stats['best_length']:.1f} -> {len(stitched_path)} grid points -> "
            f"smoothed {smooth.total_length:.2f}m, r_min={self.min_turn_radius_m:.2f}m "
            f"({len(rounded)} corners rounded{limited_note}{unrounded_note}{astar_note})"
        )

    def _subcell_to_meters(self, row, col):
        """subcell (row, col) 중심의 실제 (x, y) 미터 좌표. y는 위에서부터
        잰다 (내부 row 규칙과 동일 -- 왼쪽 아래 원점으로 뒤집는 건 축 표시와
        화면 표기에서만 한다)."""
        return (
            (col + 0.5) * self.subcell_size_m,
            (row + 0.5) * self.subcell_size_m,
        )

    def _meters_to_pixel(self, x, y):
        scale = self.subcell_px / self.subcell_size_m
        return (self.origin_x + x * scale, self.origin_y + y * scale)

    def _invalidate_coverage_path(self):
        if self.show_coverage_path:
            self.show_coverage_path = False
            self.coverage_path = None
            self.cell_markers = None
            self.smooth_path = None
            self.coverage_stats = None
            self.anim_progress = 0.0
            self.animating = False

    def _update_animation(self, dt):
        if not self.animating or not self.smooth_path:
            return
        total = self.smooth_path.total_length
        self.anim_progress += self.anim_speed * dt
        if self.anim_progress >= total:
            self.anim_progress = total
            self.animating = False

    # ---------- 저장 / 불러오기 ----------

    def save_map(self):
        os.makedirs(MAP_DIR, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"map_{timestamp}.json"
        filepath = os.path.join(MAP_DIR, filename)

        obstacle_cells = [
            [r, c]
            for r in range(self.rows)
            for c in range(self.cols)
            if self.obstacles[r][c]
        ]

        data = {
            "rows": self.rows,
            "cols": self.cols,
            "subcell_size_m": self.subcell_size_m,
            "width_m": self.cols * self.subcell_size_m,
            "height_m": self.rows * self.subcell_size_m,
            "obstacles": obstacle_cells,
        }

        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        self._set_status(f"Saved: {MAP_DIR}/{filename}")

    def save_path(self):
        """Coverage Path로 계산된 스무딩(곡률 제약 반영) 경로를
        path_point_spacing_m 간격으로 리샘플링해서 path/ 폴더에 Excel(.xlsx)
        파일로 저장한다. 좌표는 화면 눈금자와 같은 왼쪽 아래 원점(y가 위로
        증가) 기준으로 저장한다."""
        if not self.smooth_path or not self.smooth_path.points:
            self._set_status("No coverage path to save - run Coverage Path first")
            return

        os.makedirs(PATH_DIR, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"path_{timestamp}.xlsx"
        filepath = os.path.join(PATH_DIR, filename)

        resampled = path_smoothing.resample_by_distance(
            self.smooth_path, self.path_point_spacing_m
        )
        height_m = self.rows * self.subcell_size_m
        # 내부 좌표는 y가 위에서 아래로 증가 -- 눈금자/사용자 좌표계(왼쪽
        # 아래가 원점)에 맞춰 y를 뒤집어서 저장한다.
        points = [(x, height_m - y, d) for x, y, d in resampled]

        wb = Workbook()
        ws = wb.active
        ws.title = "path"
        ws.append(["index", "x_m", "y_m", "dist_along_path_m"])
        for i, (x, y, d) in enumerate(points):
            ws.append([i, x, y, d])

        meta = wb.create_sheet("meta")
        meta.append(["rows", self.rows])
        meta.append(["cols", self.cols])
        meta.append(["subcell_size_m", self.subcell_size_m])
        meta.append(["width_m", self.cols * self.subcell_size_m])
        meta.append(["height_m", height_m])
        meta.append(["point_spacing_m", self.path_point_spacing_m])
        meta.append(["num_points", len(points)])
        meta.append(["total_length_m", self.smooth_path.total_length])

        wb.save(filepath)

        self._set_status(
            f"Saved path ({len(points)} pts @ {self.path_point_spacing_m:.2f}m): "
            f"{PATH_DIR}/{filename}"
        )

    def list_saved_entries(self):
        entries = []
        if not os.path.isdir(MAP_DIR):
            return entries
        for filename in sorted(os.listdir(MAP_DIR)):
            if filename.lower().endswith(".json") and self._map_size_matches(filename):
                entries.append((MAP_DIR, filename))
        return entries

    def _map_size_matches(self, filename):
        filepath = os.path.join(MAP_DIR, filename)
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            return False
        return data.get("rows") == self.rows and data.get("cols") == self.cols

    def load_map_from_file(self, directory, filename):
        filepath = os.path.join(directory, filename)
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)

        loaded_rows = data.get("rows", self.rows)
        loaded_cols = data.get("cols", self.cols)

        if loaded_rows != self.rows or loaded_cols != self.cols:
            self._set_status(
                f"Grid size mismatch ({loaded_rows}x{loaded_cols}), load skipped"
            )
            return

        self.obstacles = [
            [False for _ in range(self.cols)] for _ in range(self.rows)
        ]
        for r, c in data.get("obstacles", []):
            self.obstacles[r][c] = True

        self._invalidate_decomposition()
        self._set_status(f"Loaded: {directory}/{filename}")

    def _set_status(self, message, duration_frames=180):
        self.status_message = message
        self.status_message_timer = duration_frames

    # ---------- Load 창(오버레이) ----------

    def open_load_overlay(self):
        self.load_overlay_files = self.list_saved_entries()
        self.load_overlay_open = True
        self.load_overlay_rects = []

    def close_load_overlay(self):
        self.load_overlay_open = False

    def draw_load_overlay(self):
        overlay_rect = pygame.Rect(
            self.screen_width // 4,
            self.screen_height // 6,
            self.screen_width // 2,
            self.screen_height * 2 // 3,
        )
        pygame.draw.rect(self.screen, COLOR_LOAD_PANEL_BG, overlay_rect, border_radius=8)
        pygame.draw.rect(self.screen, COLOR_TEXT, overlay_rect, width=2, border_radius=8)

        title_surf = self.font.render("Select a map to load (Esc to close)", True, COLOR_TEXT)
        self.screen.blit(title_surf, (overlay_rect.x + 10, overlay_rect.y + 10))

        self.load_overlay_rects = []
        item_h = 28
        start_y = overlay_rect.y + 40

        if not self.load_overlay_files:
            empty_surf = self.small_font.render(
                "(no saved maps found)", True, COLOR_TEXT
            )
            self.screen.blit(empty_surf, (overlay_rect.x + 10, start_y))
            return

        mouse_pos = pygame.mouse.get_pos()
        for i, (directory, filename) in enumerate(self.load_overlay_files):
            item_rect = pygame.Rect(
                overlay_rect.x + 5,
                start_y + i * item_h,
                overlay_rect.width - 10,
                item_h - 2,
            )
            if item_rect.bottom > overlay_rect.bottom - 5:
                break
            hovered = item_rect.collidepoint(mouse_pos)
            if hovered:
                pygame.draw.rect(self.screen, COLOR_LOAD_ITEM_HOVER, item_rect, border_radius=4)
            display_name = f"{directory}/{filename}"
            text_surf = self.small_font.render(display_name, True, COLOR_TEXT)
            self.screen.blit(text_surf, (item_rect.x + 5, item_rect.y + 4))
            self.load_overlay_rects.append((item_rect, directory, filename))

    def handle_load_overlay_click(self, pos):
        for item_rect, directory, filename in self.load_overlay_rects:
            if item_rect.collidepoint(pos):
                self.load_map_from_file(directory, filename)
                self.close_load_overlay()
                return

    # ---------- 그리기 ----------

    def _cell_rect(self, row, col):
        return pygame.Rect(
            self.origin_x + col * self.subcell_px,
            self.origin_y + row * self.subcell_px,
            self.subcell_px, self.subcell_px,
        )

    def _cell_center(self, row, col):
        return (
            self.origin_x + col * self.subcell_px + self.subcell_px / 2,
            self.origin_y + row * self.subcell_px + self.subcell_px / 2,
        )

    def draw_axes(self):
        """그리드 왼쪽(y)/아래쪽(x)에 0.5m 간격 눈금과 숫자, 축 이름을 그린다.
        좌표계는 왼쪽 아래가 (0, 0)이고 y는 위로 증가한다."""
        px_per_m = self.subcell_px / self.subcell_size_m
        step_m = 0.5
        width_m = self.cols * self.subcell_size_m
        height_m = self.rows * self.subcell_size_m
        left = self.origin_x
        top = self.origin_y
        bottom = self.origin_y + self.grid_height
        tick_len = 6

        # x축 (아래쪽)
        m = 0.0
        while m <= width_m + 1e-9:
            x = left + m * px_per_m
            pygame.draw.line(self.screen, COLOR_AXIS_TICK, (x, bottom), (x, bottom + tick_len), 2)
            label = self.tiny_font.render(f"{m:g}", True, COLOR_AXIS_TEXT)
            self.screen.blit(label, label.get_rect(midtop=(x, bottom + tick_len + 3)))
            m += step_m

        # y축 (왼쪽): 화면 위에서 m만큼 내려온 위치의 좌표값은 height_m - m
        m = 0.0
        while m <= height_m + 1e-9:
            y = top + m * px_per_m
            pygame.draw.line(self.screen, COLOR_AXIS_TICK, (left - tick_len, y), (left, y), 2)
            label = self.tiny_font.render(f"{height_m - m:g}", True, COLOR_AXIS_TEXT)
            self.screen.blit(label, label.get_rect(midright=(left - tick_len - 5, y)))
            m += step_m

        x_title = self.axis_title_font.render("x [m]", True, COLOR_AXIS_TEXT)
        self.screen.blit(x_title, x_title.get_rect(midtop=(left + self.grid_width / 2, bottom + 32)))
        y_title = pygame.transform.rotate(self.axis_title_font.render("y [m]", True, COLOR_AXIS_TEXT), 90)
        self.screen.blit(y_title, y_title.get_rect(center=(left - 52, top + self.grid_height / 2)))

    def draw_grid(self):
        grid_rect = pygame.Rect(self.origin_x, self.origin_y, self.grid_width, self.grid_height)
        self.screen.fill(COLOR_BG, grid_rect)

        if self.show_decomposition and self.labels is not None:
            for r in range(self.rows):
                for c in range(self.cols):
                    label = self.labels[r][c]
                    if label >= 0:
                        color = self.palette[label % len(self.palette)]
                        pygame.draw.rect(self.screen, color, self._cell_rect(r, c))

        for r in range(self.rows):
            for c in range(self.cols):
                if self.obstacles[r][c]:
                    pygame.draw.rect(self.screen, COLOR_OBSTACLE, self._cell_rect(r, c))

        for c in range(1, self.cols):
            x = self.origin_x + c * self.subcell_px
            pygame.draw.line(self.screen, COLOR_GRID_LINE, (x, self.origin_y), (x, self.origin_y + self.grid_height))
        for r in range(1, self.rows):
            y = self.origin_y + r * self.subcell_px
            pygame.draw.line(self.screen, COLOR_GRID_LINE, (self.origin_x, y), (self.origin_x + self.grid_width, y))

        if self.show_coverage_path and self.smooth_path:
            self._draw_coverage_path()
        else:
            center = self._cell_center(*self.start_pos)
            radius = self.subcell_px * 0.28
            pygame.draw.circle(self.screen, COLOR_START_FILL, center, radius)
            pygame.draw.circle(self.screen, COLOR_START_MARKER, center, radius, width=2)
            label = self.tiny_font.render("Start", True, COLOR_START_MARKER)
            self.screen.blit(label, label.get_rect(center=center))

        # 작업 공간 테두리
        pygame.draw.rect(self.screen, COLOR_GRID_BORDER, grid_rect.inflate(2, 2), width=2)

        self.draw_axes()

    def _draw_coverage_path(self):
        smooth = self.smooth_path

        # 스무딩 전 원래 그리드 경로를 참고용으로 흐린 점선으로 먼저 그린다.
        raw_points = [self._cell_center(r, c) for r, c in self.coverage_path]
        for p0, p1 in zip(raw_points, raw_points[1:]):
            self._draw_dashed_line(COLOR_RAW_PATH, p0, p1)

        # 스무딩으로 둥글리지 못한 모서리 (너무 급함 --
        # path_smoothing.MIN_USEFUL_RADIUS_RATIO 참고).
        for x, y in self._unrounded_corner_points():
            px, py = self._meters_to_pixel(x, y)
            pygame.draw.circle(self.screen, COLOR_UNROUNDED_CORNER, (px, py), 6, width=2)

        # 지금까지 이동한, 곡률 제약이 반영된 스무딩 경로.
        dist = self.anim_progress
        trail_px = [self._meters_to_pixel(*p) for p in self._smooth_points_up_to(dist)]
        if len(trail_px) > 1:
            pygame.draw.lines(self.screen, COLOR_PATH_LINE, False, trail_px, width=3)
            for i in range(len(trail_px) - 1):  # 원형 이음새로 굵은 선의 각진 틈을 메움
                pygame.draw.circle(self.screen, COLOR_PATH_LINE, trail_px[i], 1.5)

        self._draw_cell_markers()

        start_px = self._meters_to_pixel(*smooth.points[0])
        pygame.draw.circle(self.screen, COLOR_PATH_START, start_px, 9, width=3)

        # 에이전트는 작업 공간 밖으로 삐져나오지 않게 그리드 영역으로 잘라서 그린다.
        ax, ay = smooth.point_at_distance(dist)
        heading = self._heading_at_distance(dist)
        self.screen.set_clip(pygame.Rect(self.origin_x, self.origin_y, self.grid_width, self.grid_height))
        self._draw_agent(*self._meters_to_pixel(ax, ay), heading)
        self.screen.set_clip(None)

        self._draw_legend()

    def _draw_dashed_line(self, color, p0, p1, dash=6, gap=5):
        length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        if length < 1e-9:
            return
        ux, uy = (p1[0] - p0[0]) / length, (p1[1] - p0[1]) / length
        d = 0.0
        while d < length:
            e = min(d + dash, length)
            pygame.draw.line(
                self.screen, color,
                (p0[0] + ux * d, p0[1] + uy * d), (p0[0] + ux * e, p0[1] + uy * e), 1,
            )
            d = e + gap

    def _draw_legend(self):
        """그리드 왼쪽 위(위 여백)에 경로 표시 기호 설명을 한 줄로 그린다."""
        items = [
            ("line", COLOR_PATH_LINE, "Smoothed path"),
            ("dash", COLOR_RAW_PATH, "Grid path"),
            ("dot", COLOR_CELL_ENTRY, "Cell entry"),
            ("dot", COLOR_CELL_EXIT, "Cell exit"),
            ("ring", COLOR_PATH_START, "Start"),
        ]
        x = self.origin_x
        cy = self.origin_y - 14
        for kind, color, text in items:
            if kind == "line":
                pygame.draw.line(self.screen, color, (x, cy), (x + 18, cy), 3)
                x += 18
            elif kind == "dash":
                self._draw_dashed_line(color, (x, cy), (x + 18, cy), dash=4, gap=3)
                x += 18
            elif kind == "dot":
                pygame.draw.circle(self.screen, color, (x + 5, cy), 5)
                x += 10
            else:
                pygame.draw.circle(self.screen, color, (x + 5, cy), 5, width=2)
                x += 10
            label = self.tiny_font.render(text, True, COLOR_AXIS_TEXT)
            self.screen.blit(label, label.get_rect(midleft=(x + 5, cy)))
            x += 5 + label.get_width() + 16

    def _draw_cell_markers(self):
        """각 셀의 진입점(빨강) / 이탈점(초록)과 방문 순서 번호를 그린다
        (agriculture 논문의 Figure 1c, 15-18과 같은 표기)."""
        if not self.cell_markers:
            return
        radius = max(5.0, self.subcell_px * 0.08)
        badge_r = 10
        for marker in self.cell_markers:
            entry_px = self._cell_center(*marker["entry"])
            exit_px = self._cell_center(*marker["exit"])
            if marker["exit"] != marker["entry"]:
                pygame.draw.circle(self.screen, (255, 255, 255), exit_px, radius + 2)
                pygame.draw.circle(self.screen, COLOR_CELL_EXIT, exit_px, radius)
            pygame.draw.circle(self.screen, (255, 255, 255), entry_px, radius + 2)
            pygame.draw.circle(self.screen, COLOR_CELL_ENTRY, entry_px, radius)

            # 방문 순서 배지: 진입점 오른쪽 위에 어두운 원 + 흰 숫자
            badge_c = (entry_px[0] + radius + badge_r - 2, entry_px[1] - radius - badge_r + 2)
            pygame.draw.circle(self.screen, COLOR_CELL_ORDER_BG, badge_c, badge_r)
            label = self.tiny_font.render(str(marker["order"]), True, COLOR_CELL_ORDER_TEXT)
            self.screen.blit(label, label.get_rect(center=badge_c))

    def _smooth_points_up_to(self, distance):
        cum = self.smooth_path.cumulative_length
        pts = self.smooth_path.points
        out = []
        for i, d in enumerate(cum):
            if d > distance:
                break
            out.append(pts[i])
        tail = self.smooth_path.point_at_distance(distance)
        if not out or tail != out[-1]:
            out.append(tail)
        return out

    def _heading_at_distance(self, distance, eps=1e-3):
        smooth = self.smooth_path
        d0 = max(0.0, distance - eps)
        d1 = min(smooth.total_length, distance + eps)
        (x0, y0), (x1, y1) = smooth.point_at_distance(d0), smooth.point_at_distance(d1)
        if abs(x1 - x0) < 1e-9 and abs(y1 - y0) < 1e-9:
            return 0.0
        return math.degrees(math.atan2(y1 - y0, x1 - x0))

    def _unrounded_corner_points(self):
        if not self.smooth_path or not self.coverage_path:
            return []
        raw_m = [self._subcell_to_meters(r, c) for r, c in self.coverage_path]
        raw_m = path_smoothing.dedupe_points(raw_m)
        return [raw_m[c.index] for c in self.smooth_path.corners if c.unrounded and c.index < len(raw_m)]

    def _draw_agent(self, x, y, heading_deg):
        """에이전트를 (x, y)를 중심으로 하는 D x D 정사각형(실제 차지 면적)으로
        그리고, 진행 방향을 향하도록 회전시킨다 -- 곡률 제약 스무딩의 효과가
        눈에 보이게 하는 부분."""
        half = self.subcell_px * 0.45
        corners = [(-half, -half), (half, -half), (half, half), (-half, half)]
        theta = math.radians(heading_deg)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        def rot(cx, cy):
            return (x + cx * cos_t - cy * sin_t, y + cx * sin_t + cy * cos_t)

        poly = [rot(cx, cy) for cx, cy in corners]

        # 반투명 footprint (아래 경로가 비쳐 보이도록)
        overlay = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        pygame.draw.polygon(overlay, (*COLOR_AGENT, COLOR_AGENT_ALPHA), poly)
        self.screen.blit(overlay, (0, 0))
        pygame.draw.polygon(self.screen, COLOR_AGENT_OUTLINE, poly, width=2)

        # 진행 방향 화살표
        tip = rot(half * 0.6, 0)
        pygame.draw.polygon(
            self.screen, COLOR_AGENT_OUTLINE,
            [tip, rot(-half * 0.25, -half * 0.3), rot(-half * 0.25, half * 0.3)],
        )

    def draw_panel(self):
        panel_rect = pygame.Rect(0, self.panel_top, self.screen_width, self.panel_height)
        pygame.draw.rect(self.screen, COLOR_PANEL_BG, panel_rect)
        pygame.draw.line(self.screen, COLOR_PANEL_BORDER, (0, self.panel_top), (self.screen_width, self.panel_top), 1)

        self.decompose_button.draw(self.screen, active=self.show_decomposition)
        self.clear_button.draw(self.screen)
        self.save_button.draw(self.screen)
        self.load_button.draw(self.screen)
        self.coverage_button.draw(self.screen, active=self.show_coverage_path)
        self.save_path_button.draw(self.screen)

        second_row_y = self.decompose_button.rect.bottom + 10

        # Live mouse position in meters -- x축 아래 오른쪽에 표시해서
        # 범례/버튼/상태 메시지와 겹치지 않게 한다.
        mouse_pos = pygame.mouse.get_pos()
        cell = self.pixel_to_cell(mouse_pos)
        if cell is not None:
            mx, my = self.pixel_to_meters(mouse_pos)
            pos_surf = self.tiny_font.render(
                f"row={cell[0]} col={cell[1]}  ({mx:.2f} m, {my:.2f} m)", True, COLOR_AXIS_TEXT
            )
            self.screen.blit(
                pos_surf,
                pos_surf.get_rect(topright=(self.origin_x + self.grid_width, self.origin_y + self.grid_height + 34)),
            )

        # 상태 메시지는 길면(예: GA 결과 요약) 창 너비에 맞춰 줄바꿈해서
        # 패널 안에 들어가는 만큼 표시한다.
        if self.status_message_timer > 0:
            line_h = self.small_font.get_linesize()
            max_w = self.screen_width - 20
            y = second_row_y
            for line in self._wrap_text(self.status_message, self.small_font, max_w):
                if y + line_h > self.screen_height:
                    break
                self.screen.blit(self.small_font.render(line, True, COLOR_TEXT), (10, y))
                y += line_h
            self.status_message_timer -= 1

    @staticmethod
    def _wrap_text(text, font, max_width):
        lines = []
        current = ""
        for word in text.split(" "):
            candidate = f"{current} {word}" if current else word
            if current and font.size(candidate)[0] > max_width:
                lines.append(current)
                current = word
            else:
                current = candidate
        if current:
            lines.append(current)
        return lines

    # ---------- 이벤트 처리 ----------

    def handle_event(self, event):
        if event.type == pygame.QUIT:
            self.running = False
            return

        if self.load_overlay_open:
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self.close_load_overlay()
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                self.handle_load_overlay_click(event.pos)
            return

        if self.decompose_button.is_clicked(event):
            self.run_decomposition()
            return

        if self.clear_button.is_clicked(event):
            self.clear_obstacles()
            return

        if self.save_button.is_clicked(event):
            self.save_map()
            return

        if self.load_button.is_clicked(event):
            self.open_load_overlay()
            return

        if self.coverage_button.is_clicked(event):
            self.compute_coverage_path()
            return

        if self.save_path_button.is_clicked(event):
            self.save_path()
            return

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            cell = self.pixel_to_cell(event.pos)
            if cell is not None:
                row, col = cell
                self.drag_paint_value = not self.obstacles[row][col]
                self.set_obstacle(row, col, self.drag_paint_value)
                self.dragging = True
                self.last_drag_cell = cell
            return

        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
            self.drag_paint_value = None
            self.last_drag_cell = None
            return

        if event.type == pygame.MOUSEMOTION and self.dragging:
            cell = self.pixel_to_cell(event.pos)
            if cell is not None and cell != self.last_drag_cell:
                if self.last_drag_cell is not None:
                    self.paint_line(self.last_drag_cell, cell, self.drag_paint_value)
                else:
                    self.set_obstacle(cell[0], cell[1], self.drag_paint_value)
                self.last_drag_cell = cell

    # ---------- 메인 루프 ----------

    def run(self):
        while self.running:
            dt = self.clock.tick(self.fps) / 1000.0

            for event in pygame.event.get():
                self.handle_event(event)

            self._update_animation(dt)

            self.screen.fill(COLOR_WINDOW_BG)
            self.draw_grid()
            self.draw_panel()

            if self.load_overlay_open:
                self.draw_load_overlay()

            pygame.display.flip()

        pygame.quit()
        sys.exit()
