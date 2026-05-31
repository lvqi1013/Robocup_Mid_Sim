from dataclasses import dataclass


@dataclass
class MatchInfo:
    stage: str
    black_team: str
    red_team: str
    timestamp: str
