import numpy as np, xml.dom.minidom
import skrf.network, skrf.calibration
from skrf.network import Network
import serial, dataclasses, os, time, itertools
import tqdm
from typing import *
from AnritsuAcdFile import AnritsuAcdFile

class AnritsuAutocal3658x:
    def __init__(self, acd_path='36585K-2F', freqs_hz=None):
        '''
        This class drives an Anritsu 3658x autocal.
        :param:acd_file: Path to AutoCalData .ACD file or model number for typical data.
        :param:freqs_hz: List of frequencies at which cal will happen or None for ACD file default.
        '''
        self.serial_port = None
        self.steps = None
        self.acd = None
        if _is_model_name(acd_path):
            self.acd = _default_acd_for_model_name(acd_path)
        elif acd_path is not None:
            self.acd = AnritsuAcdFile(acd_path)
        if self.acd is not None:
            self.steps = self.acd.get_steps(freqs_hz)

    def _is_model_name(self, acd_file):
        return acd_file in ['36585K-2F']

    def _default_acd_for_model_name(self, model_name, freqs_hz):
        if model_name=="36585K-2F":
            path = os.path.join(os.path.dirname(__file__), "36585K-2F.acd")
            return AnritsuAcdFile(path)
        raise ValueError(f"{model_name} is not a known .ACD model name.")
    
    def get_steps_measured(self):
        return self.steps
    
    def get_steps_ideal(self):
        sample_network = self.steps.values().__iter__().__next__()
        return self.acd.get_steps(freqs_hz=sample_network.f)
    
    def get_freqs_hz(self):
        if self.steps:
            sample_network = self.steps.values().__iter__().__next__()
            return sample_network.f
        return self.acd.freqs_hz
    
    def try_read_from_dir(self):
        if not os.path.exists(self.dirname):
            return False
        flist = [fn for fn in os.listdir(self.dirname) if fn[-4:].lower()=='.s2p']
        self.steps = {}
        for f in flist:
            n = skrf.network.Network(f'{self.dirname}/{f}')
            self.steps[n.name] = n
    
    def measure(self, vna, freqs_hz=None, serial_path='/dev/ttyUSB0'):
        pass
        
    def _measure_8510(self, hp8510, freqs_hz=None, serial_path='/dev/ttyUSB0'):
        '''
        Use vna to measure the autocal in all states.
        This method is specialized to work with HP8510.
        Takes 26 minutes for 36585K (naive version takes 40).
        '''
        serial_port = serial.Serial(
            serial_path,
            4800,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=0,
            rtscts=False)
        out_dir = self.dirname #'36585_meas_08'
        if not freqs_hz:
            freqs_hz = self.get_freqs_hz()
        freqs_hz = np.array(freqs_hz)
        vna_hz = freqs_hz[(hp8510.min_hz<=freqs_hz) & (freqs_hz<=hp8510.max_hz)]
        sweep_plan = skrf.vi.vna.hp8510c_sweep_plan.SweepPlan.from_hz(vna_hz)
        t0 = time.time()
        rslts = {} # AutocalStep.name -> [two_port1, two_port2, ...]
        last_sweep_sec = None
        for sweep_sec,ast in tqdm.tqdm([(s,a) for s in sweep_plan.sections for a in autocal_steps]):
            if sweep_sec is not last_sweep_sec:
                sweep_sec.apply_8510(hp8510)
                last_sweep_sec = sweep_sec
            serial_port.write(bytes(ast.cmd,'utf8'))
            tp = hp8510.two_port()
            ss_hz = sweep_sec.get_hz()  # Frequencies from the 8510 are rounded, ugh.
            tp.frequency = skrf.Frequency.from_f(ss_hz,'hz')
            tp.name = ast.name
            if rslts.get(ast.name) is None:
                rslts[ast.name] = tp
            else:
                stitched = skrf.network.stitch(rslts[ast.name],tp)
                rslts[ast.name] = stitched
        self.steps = rslts
        for k,v in rslts.items():
            v.write_touchstone(dir=out_dir)
        t1 = time.time()
        print(f"    Time: {(t1-t0)/60} minutes")

    def get_cal(self):
        if not self.steps:
            print("Attempted to get cal, but no cal was loaded! ")
        assert self.steps
        meas_steps = self.steps
        a_measured_net = self.steps.values().__iter__().__next__()
        dat_steps = self.acd.get_steps(a_measured_net.f)
        assert dat_steps
        assert meas_steps
        names_acd = set(dat_steps.keys())
        names_meas = set(meas_steps.keys())
        #names = names_acd.intersection(names_meas)
        names = ['Short Short', 'Open  Short', 'Short Open', 'Load1 Short', 'Short Load1', 'Load2 Short', 'Short Load2', 'Load3 Short', 'Short Load3', 'Thru1', 'Thru2', 'Thru3', 'Thru4']
        ideals = [dat_steps[n] for n in names]
        measured = [meas_steps[n] for n in names]
        n_thrus = 4
        #n_thrus = np.sum([aa.tname is not None for aa in autocal_steps])
        return skrf.calibration.TwelveTerm(measured, ideals, n_thrus=n_thrus)
        