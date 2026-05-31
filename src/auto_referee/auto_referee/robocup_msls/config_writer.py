import json
import os
from datetime import datetime

from .models import MatchInfo


def write_match_config(match: MatchInfo) -> str:
    stem = f"match_config_{datetime.now().strftime('%Y%m%d_%H%M')}"
    path = os.path.join(os.getcwd(), f"{stem}.json")
    index = 2
    while os.path.exists(path):
        path = os.path.join(os.getcwd(), f"{stem}_{index}.json")
        index += 1

    data = {
        "stage": match.stage,
        "black_team": match.black_team,
        "red_team": match.red_team,
        "timestamp": match.timestamp,
    }
    with open(path, "w", encoding="utf-8") as fp:
        json.dump(data, fp, ensure_ascii=False, indent=2)
    return path
