"""
Code is adapted from the 6TiSCH simulator, see https://bitbucket.org/6tisch/simulator/src/master/
"""

import math
import random

from past.utils import old_div


class PisterHackModel(object):

    PISTER_HACK_LOWER_SHIFT  =         40 # dB
    TWO_DOT_FOUR_GHZ         = 2400000000 # Hz
    SPEED_OF_LIGHT           =  299792458 # m/s

    TX_POWER = 0  # dBm
    ANTENNA_GAIN = 0  # dBi
    NOISE_POWER = -105  # dBm

    # RSSI and PDR relationship obtained by experiment; dataset was available
    # at the link shown below:
    # http://wsn.eecs.berkeley.edu/connectivity/?dataset=dust
    RSSI_PDR_TABLE = {
        -97:    0.0000,  # this value is not from experiment
        -96:    0.1494,
        -95:    0.2340,
        -94:    0.4071,
        # <-- 50% PDR is here, at RSSI=-93.6
        -93:    0.6359,
        -92:    0.6866,
        -91:    0.7476,
        -90:    0.8603,
        -89:    0.8702,
        -88:    0.9324,
        -87:    0.9427,
        -86:    0.9562,
        -85:    0.9611,
        -84:    0.9739,
        -83:    0.9745,
        -82:    0.9844,
        -81:    0.9854,
        -80:    0.9903,
        -79:    1.0000,  # this value is not from experiment
    }

    @staticmethod
    def compute_mean_rssi(distance):
        # sqrt and inverse of the free space path loss (fspl)
        free_space_path_loss = (
            old_div(PisterHackModel.SPEED_OF_LIGHT,
            (4 * math.pi * distance * PisterHackModel.TWO_DOT_FOUR_GHZ))
        )

        # simple friis equation in Pr = Pt + Gt + Gr + 20log10(fspl)
        pr = (
            PisterHackModel.TX_POWER     +
            PisterHackModel.ANTENNA_GAIN +
            PisterHackModel.ANTENNA_GAIN +
            (20 * math.log10(free_space_path_loss))
        )

        # according to the receiver power (RSSI) we can apply the Pister hack model.
        # choosing the "mean" value
        return pr - old_div(PisterHackModel.PISTER_HACK_LOWER_SHIFT, 2)

    @staticmethod
    def compute_rssi(distance):
        """Compute RSSI between the points of a and b using Pister Hack"""
        # compute the mean RSSI (== friis - 20)
        mu = PisterHackModel.compute_mean_rssi(distance)

        # the receiver will receive the packet with an rssi uniformly
        # distributed between friis and (friis - 40)
        rssi = (
            mu +
            random.uniform(
                old_div(-PisterHackModel.PISTER_HACK_LOWER_SHIFT,2),
                old_div(+PisterHackModel.PISTER_HACK_LOWER_SHIFT,2)
            )
        )

        return rssi

    @staticmethod
    def convert_rssi_to_pdr(rssi):
        minRssi = min(PisterHackModel.RSSI_PDR_TABLE.keys())
        maxRssi = max(PisterHackModel.RSSI_PDR_TABLE.keys())

        if rssi < minRssi:
            pdr = 0.0
        elif rssi > maxRssi:
            pdr = 1.0
        else:
            floor_rssi = int(math.floor(rssi))
            pdr_low    = PisterHackModel.RSSI_PDR_TABLE[floor_rssi]
            pdr_high   = PisterHackModel.RSSI_PDR_TABLE[floor_rssi + 1]
            # linear interpolation
            pdr = (pdr_high - pdr_low) * (rssi - float(floor_rssi)) + pdr_low

        return pdr
