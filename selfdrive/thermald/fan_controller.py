#!/usr/bin/env python3
from abc import ABC, abstractmethod

from openpilot.common.realtime import DT_TRML
from openpilot.common.numpy_fast import interp
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.pid import PIDController

from openpilot.common.params import Params

class BaseFanController(ABC):
  @abstractmethod
  def update(self, cur_temp: float, ignition: bool) -> int:
    pass


class TiciFanController(BaseFanController):
  def __init__(self) -> None:
    super().__init__()
    cloudlog.info("Setting up TICI fan handler")

    self.last_ignition = False
    self.controller = PIDController(k_p=0, k_i=4e-3, k_f=1, rate=(1 / DT_TRML))

    # 风扇静音
    params = Params()
    self.queit_fan = False
    fire_the_baby_sitter = params.get_bool("FireTheBabysitter")
    if fire_the_baby_sitter:
      self.queit_fan = params.get_bool("QueitFan")

  def update(self, cur_temp: float, ignition: bool) -> int:
    self.controller.neg_limit = -(100 if ignition else 30)
    self.controller.pos_limit = -(30 if ignition else 0)

    if ignition != self.last_ignition:
      self.controller.reset()

    # 风扇静音
    if self.queit_fan:
      fan_pwr_out = int(interp(cur_temp, [60.0, 80.0], [0, 65]))
      # 确保风扇功率在 0 到 65 之间
      fan_pwr_out = max(0, min(65, fan_pwr_out))
    else:
      # 风扇全速工作
      error = 70 - cur_temp
      fan_pwr_out = -int(self.controller.update(
                        error=error,
                        feedforward=interp(cur_temp, [60.0, 100.0], [0, -100])
                      ))

    self.last_ignition = ignition
    return fan_pwr_out

