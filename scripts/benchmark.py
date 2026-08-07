import argparse
import json
import os
import shutil
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET_DIR = REPO_ROOT / "datasets"


def get_test_configs(dataset_dir):
    framebuffer_dim = 1024

    base_config = {
        "arch-name": "TRaX",
        "scene-name": "sponza",
        "dataset-dir": str(dataset_dir),
        "framebuffer-width": framebuffer_dim,
        "framebuffer-height": framebuffer_dim,
        "pregen-rays": 1,
        "pregen-bounce": 0,
    }

    test_scenes = ["crytek-sponza", "intel-sponza", "hairball", "bistro", "san-miguel"]
    test_bounce_types = [0, 1, 2, 3]
    bvh_presets = [1]

    configs = []
    for scene in test_scenes:
        for bounce_type in test_bounce_types:
            for merging in [0]:
                for bvh_preset in bvh_presets:
                    config = base_config.copy()
                    config["bvh-preset"] = bvh_preset
                    config["bvh-merging"] = merging
                    config["scene-name"] = scene
                    config["pregen-bounce"] = bounce_type
                    configs.append(config)

    return configs


def format_config_as_args(config):
    args = []
    for key, value in config.items():
        if isinstance(value, dict):
            value = json.dumps(value)
        args.append(f"--{key}={value}")
    return args


def generate_log_filename(config, log_dir="logs"):
    log_path = Path(log_dir)
    log_path.mkdir(parents=True, exist_ok=True)

    filename = f"{config['arch-name']}-{config['scene-name']}-bounce{config['pregen-bounce']}-m{config['bvh-merging']}-bvh{config['bvh-preset']}.log"
    return log_path / filename


def generate_image_filename(config, output_dir="images"):
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    filename = f"{config['arch-name']}-{config['scene-name']}-bounce{config['pregen-bounce']}-m{config['bvh-merging']}-bvh{config['bvh-preset']}.png"
    return output_path / filename


def move_output_image(config, output_path):
    target_image_path = generate_image_filename(config)
    if output_path.exists():
        shutil.move(str(output_path), str(target_image_path))
        print(f"Image saved as: {target_image_path}")
    else:
        print(f"Output image not found at: {output_path}")


def find_program(configuration):
    runtime_dirs = {
        "Debug": REPO_ROOT / "build" / "debug",
        "Release": REPO_ROOT / "build" / "release",
    }
    configurations = [configuration] if configuration else ["Release", "Debug"]
    executable_names = ["arches.exe"] if os.name == "nt" else ["arches"]

    candidates = []
    for config in configurations:
        build_root = runtime_dirs[config]
        for executable_name in executable_names:
            candidates.append(build_root / executable_name)

    for candidate in candidates:
        if candidate.exists():
            return candidate

    candidate_list = "\n".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f"Executable not found. Checked:\n{candidate_list}")


def execute_program(program_path, configs, working_directory, output_image="arches-out.png"):
    original_directory = Path.cwd()
    os.chdir(working_directory)

    try:
        for idx, config in enumerate(configs):
            args = format_config_as_args(config)
            cmd = [str(program_path)] + args
            log_file = generate_log_filename(config)

            print(f"[{idx + 1}/{len(configs)}] Executing: {' '.join(cmd)}")
            print(f"Logs will be saved to: {log_file}")

            with log_file.open("w") as log:
                try:
                    subprocess.run(cmd, stdout=log, stderr=log, check=True, shell=False)
                    move_output_image(config, Path(output_image))
                except subprocess.CalledProcessError as e:
                    log.write(f"\nError: Command failed with exit code {e.returncode}\n")
                except Exception as ex:
                    log.write(f"\nUnexpected error occurred: {str(ex)}\n")
    finally:
        os.chdir(original_directory)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--configuration", choices=["Debug", "Release"], default="Release", help="Build configuration to run.")
    parser.add_argument("--dataset-dir", default=str(DEFAULT_DATASET_DIR), help="Dataset directory to pass to arches.")
    args = parser.parse_args()

    dataset_dir = Path(args.dataset_dir).resolve()
    if not dataset_dir.exists():
        raise FileNotFoundError(f"Dataset directory not found: {dataset_dir}")

    program_path = find_program(args.configuration)
    execute_program(program_path, get_test_configs(dataset_dir), program_path.parent)


if __name__ == "__main__":
    main()
