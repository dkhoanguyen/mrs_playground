#!/usr/bin/python3


import argparse
from mr_herding_cbf import run_cbf
from mr_herding_apf import run_apf


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--name', type=str)
    parser.add_argument('-g', '--gui',
                        help='Render gui', action='store_true')
    args = parser.parse_args()

    for i in range(60):
        run_cbf(i, args.gui)
        run_apf(i, args.gui)


if __name__ == '__main__':
    main()
