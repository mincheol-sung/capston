import argparse
import sys

from global_planner.theta_star import planner as theta_star_planner


PLANNERS = {
    "theta_star": theta_star_planner.plan,
}


def normalize_planner_name(name):
    return name.replace("-", "_").replace("*", "_star")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Global planner dispatcher for waypoint generation."
    )
    parser.add_argument(
        "map",
        nargs="?",
        default="my_factory_map",
        help="Map name under maps/<map_name>, or a direct path to a map yaml.",
    )
    parser.add_argument(
        "--planner",
        default="theta_star",
        help="Planner implementation to use. Example: theta_star.",
    )
    return parser.parse_known_args(argv)


def main(argv=None):
    args, remaining_args = parse_args(argv)
    planner_name = normalize_planner_name(args.planner)
    planner_fn = PLANNERS.get(planner_name)
    if planner_fn is None:
        available = ", ".join(sorted(PLANNERS))
        print(f"ERROR: unknown planner '{args.planner}'. Available: {available}")
        return 2

    result = planner_fn([args.map] + remaining_args)
    if not result:
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
