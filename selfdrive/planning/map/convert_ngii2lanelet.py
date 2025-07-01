import json
import argparse
import os
import importlib

from ngii2lanelet import NGII2LANELET


def main(args):
    lanelet = NGII2LANELET(
        folder_path=args.ngii_path,
        precision=args.precision,
        base_lla=args.base_lla,
        is_utm=args.is_utm
        )

    name = args.ngii_path.split('/')[-1]

    with open('%s.json'%(name), 'w', encoding='utf-8') as f:
        json.dump(lanelet.map_data, f, indent="\t")

    with open('%s_ID.json'%(name), 'w', encoding='utf-8') as f:
        json.dump(lanelet.link_id_data, f, indent="\t")

    remove_lanelet_pickle(name)

def remove_lanelet_pickle(map_name: str) -> None:
    viz_pkg = importlib.import_module('selfdrive.visualize')
    viz_dir = os.path.dirname(os.path.abspath(viz_pkg.__file__))

    pickle_dir = os.path.join(viz_dir, 'pickles')
    pkl_path   = os.path.join(pickle_dir, f'{map_name}_lanelet_graph.pkl')

    try:
        os.remove(pkl_path)
        print(f"[INFO] '{pkl_path}' 삭제 완료.")
    except FileNotFoundError:
        print(f"[INFO] 삭제할 파일이 없습니다: '{pkl_path}'")
    except PermissionError:
        print(f"[ERROR] 파일을 삭제할 권한이 없습니다: '{pkl_path}'")
    except OSError as e:
        print(f"[ERROR] 알 수 없는 오류로 삭제 실패: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    base_dict = {
    		"KCity" : (37.2292221592864, 126.76912499027308, 29.18400001525879),
    		"songdo" : (37.39657805498484, 126.6321430873685, 7.369),
    		"songdo-testbed" : (37.4179788, 126.6140342, 7),
    		"songdo_campus" : (37.383333, 126.656111, -0.51)
    		}

    name = 'songdo_campus'

    parser.add_argument('--ngii_path', type=str, default='./%s'%(name))
    parser.add_argument('--precision', type=float, default=1.0)
    parser.add_argument('--base_lla', type=tuple, default=base_dict[name], help='(lat, lon, alt)')
    parser.add_argument('--is_utm', type=bool, default=False)  # KCity 일때만 True인가 그럼

    args = parser.parse_args()

    main(args)