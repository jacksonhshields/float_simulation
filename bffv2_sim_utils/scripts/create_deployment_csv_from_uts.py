#!/usr/bin/env python3

import yaml
import pandas as pd
import argparse
import glob
import os
import pymap3d
import ast
import numpy as np
def get_args():
    parser = argparse.ArgumentParser(description="Writes a csv for the float deployment locations")
    parser.add_argument("--input", "-i", type=str, help="The path to .csv")
    parser.add_argument("--datum", "-d", type=str, help="Datum")
    parser.add_argument("--dive-time", "-t", required=True, type=int, help="The path to the .yaml or .json configuration file that was used to plan the missions")
    parser.add_argument("--output", "-o", type=str, required=True, help="Output csv for deployments")
    args = parser.parse_args()
    return args

def main(args):
    df = pd.read_csv(args.input)
    datum = ast.literal_eval(args.datum)
    outdata = {k:[] for k in ['deployment_name','lat','lon','dive_time','float_name','x','y']}
    for index, row in df.iterrows():
        lat = row['Lat']
        lon = row['Lon']
        enu = np.array(pymap3d.geodetic2enu(lat=lat, lon=lon, h=0., lat0=datum[0], lon0=datum[1], h0=0.))
        outdata['deployment_name'].append(int(index))
        outdata['lat'].append(lat)
        outdata['lon'].append(lon)
        outdata['dive_time'].append(args.dive_time)
        outdata['float_name'].append('float2')
        outdata['x'].append(enu[0])
        outdata['y'].append(enu[1])
    outdf = pd.DataFrame.from_dict(outdata)
    outdf.to_csv('deploy_set.csv', index=False, header=False)



if __name__ == "__main__":
    main(get_args())