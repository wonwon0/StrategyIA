# Under MIT License, see LICENSE.txt

from RULEngine.Debug.ui_debug_command import UIDebugCommand
from RULEngine.Util.Pose import Pose, Position
from RULEngine.Util.game_world import GameWorld
from ai.executors.executor import Executor
from ai.states.world_state import WorldState


class DebugExecutor(Executor):

    def __init__(self, p_world_state: WorldState):
        super().__init__(p_world_state)
        self.debug_in = None
        self.our_transformed_incoming_debug = []

    def exec(self)->None:
        for command in self.debug_in:
            self._parse_command(UIDebugCommand(command))

    def set_reference(self, world_reference: GameWorld)->None:
        self.debug_in = world_reference.debug_info

    def _parse_command(self, cmd: UIDebugCommand)->None:
        if cmd.is_tactic_cmd():
            self._parse_tactic(cmd)

    def _parse_tactic(self, cmd: UIDebugCommand)->None:
        # probably useless
        player_id = self._sanitize_pid(cmd.data['id'])

        # required to read the packet from the UIDebug
        tactic_name = cmd.data['tactic']
        target = cmd.data['target']
        target = Pose(Position(target[0], target[1]))
        # Default tactic if something goes wrong
        tactic = self.ws.play_state.get_new_tactic('Idle')(self.ws.game_state,
                                                           player_id,
                                                           target)
        try:
            # create the new tactic
            tactic = self.ws.play_state.get_new_tactic(tactic_name)\
                (self.ws.game_state, player_id, target)
        except Exception as e:
            print(e)
            print("La tactique n'a pas été appliquée par "
                  "cause de mauvais arguments.")

        # put the new tactic in
        self.ws.play_state.current_tactic[player_id] = tactic

    @staticmethod
    def _sanitize_pid(pid: int)->int:
        if 0 <= pid < 6:
            return pid
        elif 6 <= pid < 12:
            return pid - 6
        else:
            return 0
