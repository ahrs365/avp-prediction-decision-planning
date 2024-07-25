import json
from typing import Dict, List, Set
import numpy as np
import os

import yaml
from yaml.loader import SafeLoader

_ROOT = os.path.abspath(os.path.dirname(__file__))

# Load parking map
with open(
    "/home/ahrs/workspace/nday/bspline_lattice_planner/data/parking_map.yml"
) as f:
    MAP_DATA = yaml.load(f, Loader=SafeLoader)

PARKING_AREAS = MAP_DATA["PARKING_AREAS"]
ENTRANCE_AREA = {"min": np.array([5, 70]), "max": np.array([25, 80])}


class Dataset:

    def __init__(self):
        self.frames = {}
        self.agents = {}
        self.instances = {}
        self.scenes = {}
        self.obstacles = {}

    def load(self, filename):
        with open(filename + "_frames.json") as f:
            self.frames.update(json.load(f))
        with open(filename + "_agents.json") as f:
            self.agents.update(json.load(f))
        with open(filename + "_instances.json") as f:
            self.instances.update(json.load(f))
        with open(filename + "_obstacles.json") as f:
            self.obstacles.update(json.load(f))
        with open(filename + "_scene.json") as f:
            scene = json.load(f)
            self.scenes[scene["scene_token"]] = scene

    def get_data(self, filename):
        self.load(filename)
        data = {
            "frames": self.frames,
            "agents": self.agents,
            "instances": self.instances,
            "obstacles": self.obstacles,
            "scenes": self.scenes,
        }
        return json.dumps(data)

    def get(self, obj_type: str, token: str) -> Dict:
        """
        Get data object as a dictionary

        `obj_type`: string, choose from ['frame', 'agent', 'instance', 'obstacle', 'scene']
        `token`: the corresponding token
        """
        assert obj_type in ["frame", "agent", "instance", "obstacle", "scene"]

        if obj_type == "frame":
            return self.frames[token]
        elif obj_type == "agent":
            return self.agents[token]
        elif obj_type == "instance":
            return self.instances[token]
        elif obj_type == "obstacle":
            return self.obstacles[token]
        elif obj_type == "scene":
            return self.scenes[token]

    def list_scenes(self) -> List[str]:
        """
        List the tokens of scenes loaded in the current dataset
        """
        return list(self.scenes.keys())

    def get_frame_at_time(self, scene_token: str, timestamp: float, tol=0.039):
        """
        Get the frame object at certain time

        `scene_token`: The scene where the frame comes from
        `timestamp`: time (float) in sec
        `tol`: typically this is the interval between frames
        """
        scene = self.get("scene", scene_token)
        frame_token = scene["first_frame"]
        while frame_token:
            frame = self.get("frame", frame_token)
            if abs(frame["timestamp"] - timestamp) < tol:
                return frame
            frame_token = frame["next"]

        assert (
            frame_token != ""
        ), "Didn't find the frame at the specified time. It may exceeds the video length."

    def get_agent_instances(self, agent_token: str) -> List[Dict]:
        """
        Return the list of instance objects for the specific agent

        `agent_token`: Token of the agent
        """
        agent_instances = []
        next_instance = self.agents[agent_token]["first_instance"]
        while next_instance:
            inst = self.instances[next_instance]
            agent_instances.append(inst)
            next_instance = inst["next"]
        return agent_instances

    def get_agent_future(self, instance_token: str, timesteps: int = 5):
        """
        Return a list of future instance objects for the same agent.

        `instance_token`: The token of the current instance
        `timesteps`: (int) Number of steps in the future.
        """
        return self._get_timeline("instance", "next", instance_token, timesteps)

    def get_agent_past(self, instance_token: str, timesteps: int = 5):
        """
        Return a list of past instance objects for the same agent.

        `instance_token`: The token of the current instance
        `timesteps`: (int) Number of steps in the past.
        """
        return self._get_timeline("instance", "prev", instance_token, timesteps)

    def get_future_frames(self, frame_token: str, timesteps: int = 5):
        """
        Return a list of future frame objects.

        `frame_token`: The token of the current frame
        `timesteps`: (int) Number of steps in the future.
        """
        return self._get_timeline("frame", "next", frame_token, timesteps)

    def get_past_frames(self, frame_token: str, timesteps: int = 5):
        """
        Return a list of past frame objects.

        `frame_token`: The token of the current frame
        `timesteps`: (int) Number of steps in the past.
        """
        return self._get_timeline("frame", "prev", frame_token, timesteps)

    def _get_timeline(self, obj_type, direction, token, timesteps) -> List[Dict]:
        if obj_type == "frame":
            obj_dict = self.frames
        elif obj_type == "instance":
            obj_dict = self.instances

        timeline = [obj_dict[token]]
        next_token = obj_dict[token][direction]
        for _ in range(timesteps):
            if not next_token:
                break
            next_obj = obj_dict[next_token]
            timeline.append(next_obj)
            next_token = next_obj[direction]

        if direction == "prev":
            timeline.reverse()

        return timeline

    def signed_speed(self, inst_token: str) -> float:
        """
        Return the speed of the current instance with sign. Positive means it is moving forward, negative measn backward.

        `inst_token`: The token of the current instance
        """
        instance = self.get("instance", inst_token)

        heading_vector = np.array(
            [np.cos(instance["heading"]), np.sin(instance["heading"])]
        )

        if instance["next"]:
            next_inst = self.get("instance", instance["next"])
        else:
            next_inst = instance

        if instance["prev"]:
            prev_inst = self.get("instance", instance["prev"])
        else:
            prev_inst = instance
        motion_vector = np.array(next_inst["coords"]) - np.array(prev_inst["coords"])

        if heading_vector @ motion_vector > 0:
            return instance["speed"]
        else:
            return -instance["speed"]

    def get_future_traj(
        self, inst_token: str, static_thres: float = 0.02
    ) -> np.ndarray:
        """
        get the future trajectory of this agent, starting from the current frame
        The static section at the begining and at the end will be truncated

        `static_thres`: the threshold to determine whether it is static. Default is 0.02m/s

        Output: T x 4 numpy array. (x, y, heading, speed). T is the time steps
        """
        traj = []

        next_token = inst_token
        while next_token:
            instance = self.get("instance", next_token)
            signed_speed = self.signed_speed(next_token)
            traj.append(
                np.array(
                    [
                        instance["coords"][0],
                        instance["coords"][1],
                        instance["heading"],
                        signed_speed,
                    ]
                )
            )

            next_token = instance["next"]

        last_idx = len(traj) - 1

        # Find the first non-static index
        idx_start = 0
        while idx_start < last_idx:
            if abs(traj[idx_start][3]) < static_thres:
                idx_start += 1
            else:
                break

        # Find the last non-static index
        idx_end = last_idx
        while idx_end > 0:
            if abs(traj[idx_end][3]) < static_thres:
                idx_end -= 1
            else:
                break

        if idx_end > idx_start:
            return np.array(traj[idx_start:idx_end])
        else:
            # If all indices are static, only return the current time step
            return traj[0].reshape((-1, 4))

    def _inside_parking_area(self, inst_token: str) -> bool:
        """
        check whether the instance is inside the parking area
        """
        instance = self.get("instance", inst_token)
        coords = np.array(instance["coords"])

        for _, area in PARKING_AREAS.items():
            bounds = np.array(area["bounds"])
            bounds_min = np.min(bounds, axis=0)
            bounds_max = np.max(bounds, axis=0)

            if all(coords > bounds_min) and all(coords < bounds_max):
                return True

        return False

    def _ever_inside_parking_area(self, inst_token: str, direction: str):
        """
        check whether the instance is ever inside the parking area

        `direction`: 'prev' - was inside the parking area before, 'next' - will go into the parking area
        """
        result = False
        next_inst_token = inst_token

        while next_inst_token and not result:
            result = self._inside_parking_area(next_inst_token)

            next_inst_token = self.get("instance", next_inst_token)[direction]

        return result

    def _will_leave_through_gate(self, inst_token: str):
        """
        check whether the instance will leave through the gate
        """
        result = False
        next_inst_token = inst_token

        while next_inst_token and not result:
            instance = self.get("instance", next_inst_token)
            coords = np.array(instance["coords"])
            result = (
                all(coords > ENTRANCE_AREA["min"])
                and all(coords < ENTRANCE_AREA["max"])
                and instance["heading"] > 0
            )

            next_inst_token = instance["next"]

        return result

    def _has_entered_through_gate(self, inst_token: str):
        """
        check whether the instance has entered through entrance
        """
        result = False
        next_inst_token = inst_token

        while next_inst_token and not result:
            instance = self.get("instance", next_inst_token)
            coords = np.array(instance["coords"])
            result = (
                all(coords > ENTRANCE_AREA["min"])
                and all(coords < ENTRANCE_AREA["max"])
                and instance["heading"] < 0
            )

            next_inst_token = instance["prev"]

        return result

    def get_inst_mode(self, inst_token: str, static_thres=0.02) -> str:
        """
        Determine the mode of the vehicle among ["parked", "incoming", "outgoing", 'unclear']. Return the mode as a string and also modify the instance object.
        """
        instance = self.get("instance", inst_token)

        if self._inside_parking_area(inst_token) and instance["speed"] < static_thres:
            mode = "parked"
        elif self._ever_inside_parking_area(inst_token, "prev"):
            mode = "outgoing"
        elif self._ever_inside_parking_area(inst_token, "next"):
            mode = "incoming"
        elif self._has_entered_through_gate(inst_token):
            mode = "incoming"
        elif self._will_leave_through_gate(inst_token):
            mode = "outgoing"
        else:
            mode = "unclear"

        instance["mode"] = mode

        return mode

    def get_inst_at_location(
        self,
        frame_token: str,
        coords: np.ndarray,
        exclude_types: Set[str] = {"Pedestrian", "Undefined"},
    ) -> Dict:
        """
        Return the closet instance object (with certain type) at a given location
        `coords`: array-like with two entries [x, y]
        `exclude_types`: the types that we don't want. The reason we exclude types is that the vehicles might have multiple types like 'Car', 'Bus', 'Truck'.
        """
        frame = self.get("frame", frame_token)
        min_dist = np.inf
        min_inst = None
        for inst_token in frame["instances"]:
            instance = self.get("instance", inst_token)
            agent = self.get("agent", instance["agent_token"])
            if agent["type"] not in exclude_types:
                x, y = instance["coords"]
                dist = (coords[0] - x) ** 2 + (coords[1] - y) ** 2
                if dist < min_dist:
                    min_dist = dist
                    min_inst = instance

        return min_inst


dataset = Dataset()


def get_dataset_data(filename):
    return dataset.get_data(filename)
