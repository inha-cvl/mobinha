import json
import argparse

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

    with open('%s_id.json'%(name), 'w', encoding='utf-8') as f:
        json.dump(lanelet.new2ori, f, indent="\t")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    name = 'KIAPI'
    # name = 'KCity' 
    # name = 'DMC'
    # name = 'AlphaCity' 

    parser.add_argument('--ngii_path', type=str, default='./%s'%(name))
    parser.add_argument('--precision', type=float, default=1.0)
    parser.add_argument('--base_lla', type=tuple, default=None, help='(lat, lon, alt)')
    parser.add_argument('--is_utm', type=bool, default=True)

    args = parser.parse_args()

    main(args)
