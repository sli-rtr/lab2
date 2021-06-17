#!/usr/bin/env python3

# read a robot.urdf inside a toolkit project zip, and update the accel and jerk limits

import argparse
import zipfile
import tempfile
import os
import shutil
from lxml import etree

def main():
    parser = argparse.ArgumentParser(description='Update joint limits for accel and jerk')
    parser.add_argument(
        "-f",
        "--file",
        help="File path to project file. Required argument.",
        type=str,
        required=True)
    parser.add_argument(
        "-a",
        "--acceleration",
        help="acceleration limit",
        type=float,
        required=True)
    parser.add_argument(
        "-j",
        "--jerk",
        help="jerk limit",
        type=float,
        required=True)

    args = parser.parse_args()
    fname = args.file
    accel = args.acceleration
    jerk = args.jerk

    if accel < 0 or jerk < 0:
        print("Error invalid inputs. Acceleration and jerk must be nonnegative.")
        return

    return parse(fname, accel, jerk)

def parse(fname, accel, jerk):
    # generate temp file
    tmpfd, tmpname = tempfile.mkstemp(dir=os.path.dirname(fname))
    os.close(tmpfd)

    urdf_path = os.path.join(os.path.splitext(os.path.basename(fname))[0],'robot.urdf')

    # create temp copy of archive w/o the robot.urdf file
    with zipfile.ZipFile(fname) as inzip, zipfile.ZipFile(tmpname, 'w') as outzip:
        for inzipinfo in inzip.infolist():
            with inzip.open(inzipinfo) as infile:
                if inzipinfo.filename != urdf_path:
                    outzip.writestr(inzipinfo.filename, infile.read())
                else:
                    # extract to a temp directory
                    tmpdname = tempfile.mkdtemp(dir=os.path.dirname(fname))
                    inzip.extract(inzipinfo.filename, tmpdname)
                    urdf_temp_path = os.path.join(tmpdname,os.path.splitext(os.path.basename(fname))[0],'robot.urdf')

                    # update robot.urdf with new accel/jerk
                    tree = etree.parse(urdf_temp_path)
                    root = tree.getroot()
                    for limit in root.findall('./joint/limit'):
                        limit.attrib['acceleration'] = str(accel)
                        limit.attrib['jerk'] = str(jerk)
                    tree.write(urdf_temp_path)

                    # write file into zip, delete temp folder
                    outzip.write(urdf_temp_path, urdf_path)
                    shutil.rmtree(tmpdname)

    # replace archive
    os.remove(fname)
    os.rename(tmpname, fname)

if __name__ == "__main__":
    main()
