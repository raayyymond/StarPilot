"""stream_content 'birds_eye': the car screen draws only the birds-eye view while driving and builds nothing else."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

from openpilot.starpilot.system.starport import identity as identity_store
from openpilot.starpilot.system.starport import projection_onroad as projection
from openpilot.starpilot.system.starport.projection_layout import CAR_EXIT, default_layout_for_viewport
from openpilot.starpilot.system.starport.render_profile import RenderSummary
from openpilot.starpilot.system.starport.tests.test_projection_onroad_boundary import FakeGraphics

VIEWPORT = (2880, 1080)
ROOT = Path(projection.__file__).parent


class FakeBirdsEye:
  def __init__(self, *, text, source):
    self.text, self.source = text, source
    self.rendered = []
    self.long_control = True
    self.panel = self.opacity = self.parent_clip = self.road_style = self.metric = None

  def render(self, rect):
    self.rendered.append((rect, self.panel, self.opacity, self.parent_clip))


class TestBirdsEyeOnly(unittest.TestCase):
  def native(self, *, road_style=None, long_control=None):
    events = []
    ui = SimpleNamespace(projection_read_only=True, started=False, sm=object(), started_frame=3, is_metric=False,
                         params=object())

    class Fonts:
      def __init__(self, profile, directory, *, headless_context):
        events.append('fonts')

      def draw(self, text, *_):
        events.append(('text', text))

      def close(self):
        events.append('fonts_closed')

    graphics = SimpleNamespace(**{name: getattr(FakeGraphics, name) for name in dir(FakeGraphics) if not name.startswith('_')})
    graphics.draw_rectangle = lambda *_: events.append('rectangle')
    graphics.Vector2 = lambda *values: values
    graphics.draw_line_ex = lambda *_: events.append('exit_button')
    # Only what the birds-eye-only screen may use: any camera/view/map/pip/feed access raises AttributeError.
    native = SimpleNamespace(rl=graphics, ui_state=ui, fonts=Fonts, font_role=SimpleNamespace(BRAND='brand', MEDIUM='medium'),
                             profile=SimpleNamespace(LARGE='large'), font_directory=lambda: Path('/unused'),
                             birds_eye=FakeBirdsEye, birds_eye_text=lambda fonts: ('text', fonts),
                             road_style=road_style or (lambda ui: {'pathMode': 'color'}),
                             long_control=long_control or (lambda ui: False))
    return native, events

  def view(self, native, layout=None, **kwargs):
    view = projection.ProjectionOnroad(dependencies=native, viewport=VIEWPORT, content='birds_eye',
                                       customization=layout or default_layout_for_viewport(VIEWPORT), **kwargs)
    self.addCleanup(view.close)
    return view

  def test_construction_builds_only_the_birds_eye_view(self):
    native, events = self.native()
    view = self.view(native)
    for name in ('camera', 'onroad', 'monitor', 'pip', 'map', 'map_feed', 'adapter', 'favorites',
                 'now_playing_feed', 'jetlink_feed', '_slc_input'):
      self.assertIsNone(getattr(view, name), name)
    self.assertIsInstance(view.birds_eye, FakeBirdsEye)
    self.assertEqual(view.birds_eye.source(), (native.ui_state.sm, 3))
    view.prepare()  # nothing to feed offroad
    view.cancel_touch()
    view.close()
    self.assertEqual(events, ['fonts', 'fonts_closed'])

  def test_unknown_content_is_refused(self):
    native, _ = self.native()
    with self.assertRaises(ValueError):
      projection.ProjectionOnroad(dependencies=native, viewport=VIEWPORT, content='camera')

  def test_driving_draws_only_the_birds_eye_view_over_the_whole_frame(self):
    native, events = self.native()
    view = self.view(native)
    native.ui_state.started = True
    view.prepare()
    view.render()
    renderer = view.birds_eye
    self.assertEqual(renderer.rendered, [((0, 0, 2880, 1080), False, 1.0, None)])
    self.assertEqual(renderer.road_style, {'pathMode': 'color'})
    self.assertFalse(renderer.metric)
    self.assertFalse(renderer.long_control)
    self.assertIsNone(view.camera_stream, 'nothing is camera-paced')
    self.assertNotIn('rectangle', events, 'no standby, border or HUD drawing')
    self.assertNotIn('exit_button', events)
    self.assertFalse([event for event in events if isinstance(event, tuple)], 'no StarPilot brand text')

  def test_birds_eye_view_yields_the_hud_areas_and_the_hud_draws_after_it(self):
    native, _ = self.native()
    order = []
    native.ui_state.CP = 'CP'

    class Hud:
      def __init__(self, text, source):
        self.text, self.source = text, source

      @staticmethod
      def view_rect(width, height):
        return 84, 0, width - 84, height - 64

      def render(self, width, height):
        order.append(('hud', len(view.birds_eye.rendered), width, height, self.source()[1]))

      def close(self):
        order.append('hud_closed')

    native.birds_eye_hud = Hud
    view = self.view(native)
    self.assertEqual(view.hud.text[0], 'text', 'the HUD shares the birds-eye bitmap text drawer')
    native.ui_state.started = True
    view.render()
    self.assertEqual(view.birds_eye.rendered[0][0], (84, 0, VIEWPORT[0] - 84, VIEWPORT[1] - 64))
    self.assertEqual(order, [('hud', 1, *VIEWPORT, 'CP')], 'drawn after the birds-eye view')
    view.close()
    self.assertEqual(order[-1], 'hud_closed')

  def test_dependencies_build_the_hud_lazily(self):
    source = (ROOT / 'projection_onroad.py').read_text()
    self.assertNotIn('birds_eye_hud import', source.split('def birds_eye_dependencies')[0], 'no module-level HUD import')
    self.assertIn('from openpilot.starpilot.system.starport.birds_eye_hud import BirdsEyeHud', source)

  def test_settings_are_read_once_a_second_and_failures_keep_defaults(self):
    reads = []

    def road_style(ui):
      reads.append('style')
      raise ValueError('bad document')
    native, _ = self.native(road_style=road_style, long_control=lambda ui: None)
    view = self.view(native)
    native.ui_state.started = True
    view.prepare()
    view.prepare()
    self.assertEqual(reads, ['style'])
    view.render()
    self.assertEqual(view.birds_eye.road_style, {})
    self.assertTrue(view.birds_eye.long_control, 'unknown longitudinal control keeps the renderer default')

  def test_standby_keeps_the_standby_screen_and_exit_button(self):
    native, events = self.native()
    focused = []
    layout = default_layout_for_viewport(VIEWPORT)
    view = self.view(native, layout, native_focus=lambda: focused.append(True))
    view.render()
    self.assertIn('rectangle', events)
    self.assertIn(('text', 'StarPilot'), events)
    self.assertIn('exit_button', events)
    self.assertEqual(view.birds_eye.rendered, [])
    placed = layout['widgets'][CAR_EXIT]
    self.tap(view, placed['x'] + 48, placed['y'] + 48)
    self.assertEqual(focused, [True])

  def tap(self, view, x, y):
    view.handle_touches([SimpleNamespace(kind='down', x=x / VIEWPORT[0], y=y / VIEWPORT[1]),
                         SimpleNamespace(kind='up', x=x / VIEWPORT[0], y=y / VIEWPORT[1])])

  def test_taps_while_driving_do_nothing(self):
    native, _ = self.native()
    focused = []
    layout = default_layout_for_viewport(VIEWPORT)
    view = self.view(native, layout, native_focus=lambda: focused.append(True))
    native.ui_state.started = True
    view.render()
    self.tap(view, 1440, 540)
    placed = layout['widgets'][CAR_EXIT]
    self.tap(view, placed['x'] + 48, placed['y'] + 48)
    self.assertEqual(focused, [], 'the exit button is not drawn while driving, so it takes no taps')
    self.assertEqual(view.birds_eye_display(), 'full')
    view.render()
    self.assertEqual(len(view.birds_eye.rendered), 2)
    self.assertTrue(all(call[1] is False for call in view.birds_eye.rendered), 'no panel/full toggle')

  def test_layout_changes_only_move_the_exit_button(self):
    native, _ = self.native()
    view = self.view(native)
    layout = default_layout_for_viewport(VIEWPORT)
    view.apply_layout(layout)
    self.assertIs(view.customization, layout)
    self.assertIsNone(view.map)

  def test_dependencies_import_nothing_the_birds_eye_view_does_not_draw(self):
    tree = ast.parse((ROOT / 'projection_onroad.py').read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'birds_eye_dependencies')
    modules = {node.module for node in ast.walk(function) if isinstance(node, ast.ImportFrom)}
    for forbidden in ('openpilot.cereal.visionipc', 'openpilot.selfdrive.ui.onroad.cameraview',
                      'openpilot.selfdrive.ui.onroad.augmented_road_view', 'openpilot.selfdrive.ui.onroad.model_renderer',
                      'openpilot.starpilot.ui.onroad', 'openpilot.starpilot.ui.onroad_map', 'openpilot.starpilot.ui.pip_render',
                      'openpilot.starpilot.ui.onroad_dm', 'openpilot.starpilot.system.starport.projection_favorites',
                      'openpilot.starpilot.system.starport.projection_now_playing', 'openpilot.starpilot.models.jetlink_status'):
      self.assertNotIn(forbidden, modules)
    self.assertIn('openpilot.selfdrive.ui.onroad.birds_eye_renderer', modules)

  def test_renderer_reads_the_mode_and_paces_birds_eye_by_the_encoder(self):
    source = (ROOT / 'current_car_ui.py').read_text()
    self.assertIn('stream_content = stream_content or config["stream_content"]', source)
    self.assertIn('content=stream_content)', source)
    self.assertIn('camera_pacer = CameraPacer() if stream_content == "full" else None', source)
    self.assertIn('print(f"car view content: {stream_content}", flush=True)', source)

  def test_preview_passes_the_content_override_to_the_renderer(self):
    from openpilot.tools.starport import preview
    self.assertIsNone(preview.parse_args([]).stream_content)
    self.assertEqual(preview.parse_args(['--stream-content', 'full']).stream_content, 'full')
    self.assertEqual(preview.renderer_command('f', 't', 'c', 'full')[-2:], ['--stream-content', 'full'])
    self.assertNotIn('--stream-content', preview.renderer_command('f', 't', 'c'))


def test_stream_content_config_defaults_and_validation(tmp_path):
  path = tmp_path / 'config.json'
  assert identity_store.load_config(path)['stream_content'] == 'birds_eye'
  path.write_text(json.dumps({'config_version': identity_store.CONFIG_VERSION, 'stream_content': 'full'}))
  assert identity_store.load_config(path)['stream_content'] == 'full'
  for bad in ('camera', '', 3, None):
    path.write_text(json.dumps({'config_version': identity_store.CONFIG_VERSION, 'stream_content': bad}))
    assert identity_store.load_config(path)['stream_content'] == 'birds_eye'


def test_render_summary_reports_phases_drops_content_and_pacing():
  summary = RenderSummary(0.0, interval=1.0, content='birds_eye')
  for began in (0.0, 0.5):
    summary.add('update', 0.001)
    summary.add('prepare', 0.0005)
    summary.add('draw', 0.010)
    summary.add('submit', 0.002)
    summary.add('readback_wait', 0.004)
    assert summary.frame_done(began, began + 0.02) is None
  summary.drop()
  report = summary.frame_done(0.98, 1.0, camera_paced=True)
  assert report == {'event': 'render_stats', 'fps': 3.0, 'frame_ms': 20.0, 'update_ms': 0.67, 'prepare_ms': 0.33,
                    'draw_ms': 6.67, 'submit_ms': 1.33, 'readback_wait_ms': 2.67, 'dropped': 1,
                    'content': 'birds_eye', 'paced': 'interval'}
  for began in (1.0, 1.5):
    summary.frame_done(began, began + 0.01, camera_paced=True)
  report = summary.frame_done(1.99, 2.0, camera_paced=True)
  assert report['paced'] == 'camera' and report['dropped'] == 0 and report['draw_ms'] == 0.0


if __name__ == '__main__':
  unittest.main()
