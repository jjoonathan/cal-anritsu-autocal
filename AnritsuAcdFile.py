import numpy as np, xml.dom.minidom
import skrf.network, skrf.calibration
from skrf.network import Network
import serial, dataclasses, os, time, itertools
import tqdm
from typing import *

@dataclasses.dataclass
class AnritsuAutocalStep:
    name     : str
    ideal    : Network = None  # Two port ideal network
    cmd      : str         # Serial command that puts the autocal into this state
    lname    : str = None  # Key to look up left (port 1) ideal in ACD
    rname    : str = None  # Key to look up right (port 2) ideal in ACD
    tname    : str = None  # Key to look up thru ideal in ACD
    measured : Network = None


class AnritsuAcdFile:
    def __init__(self, fname):
        if fname is not None:
            self._read_file(fname)
    
    def _read_file(fname):
        self.acd = acd = xml.dom.minidom.parse(fname)
        self.version = acd.getElementsByTagName('Version')[0].childNodes[0].wholeText.strip()
        self.serial  = acd.getElementsByTagName('Serial')[0].childNodes[0].wholeText.strip()
        self.module_type = acd.getElementsByTagName('Module.Type')[0].childNodes[0].wholeText.strip()
        self.assurance_true = acd.getElementsByTagName('Assurance.True')[0].childNodes[0].wholeText.strip()
        self.start_hz = int(acd.getElementsByTagName('Start')[0].childNodes[0].wholeText.strip())
        self.stop_hz = int(acd.getElementsByTagName('Stop')[0].childNodes[0].wholeText.strip())
        #self.deltaf = acd.getElementsByTagName('Deltaf')[0].childNodes[0].wholeText.strip()
        self.num_pts = int(acd.getElementsByTagName('NumberPts')[0].childNodes[0].wholeText.strip())
        self.freqs_hz = [float(s) for s in acd.getElementsByTagName('FreqList')[0].childNodes[0].wholeText.split('\n') if len(s)>0]
    
    def get_freqs_hz(self):
        ''' Get the frequnecies at which the ACD file contains ideal measurements. '''
        return self.freqs_hz
    
    def get_steps(self, freqs_hz=None):
        '''
        Main method. Get the ideal measurements stored in the ACD file.
        :param freqs_hz: List of frequencies the ideal networks should be interpolated to. Default: use list from ACD file.
        :return: A new list of AnritsuAutocalStep
        '''
        ret = {}
        steps = _new_steps_for_module_type(self.module_type)
        for step in steps:
            is_thru_step = step.tname is not None
            is_reflect_step = not is_thru_step
            if is_thru_step:
                two_port_network = self._two_port_from_4param(
                    f'{step.tname}.11',
                    f'{step.tname}.12',
                    f'{step.tname}.21',
                    f'{step.tname}.22')
            elif is_reflect_step:
                two_port_network = self._two_port_from_reflective(
                    acs.lname,
                    acs.rname)
            two_port_network.name = step.name
            should_interpolate_network = freqs_hz is not None
            if should_interpolate_network:
                interp_f = skrf.Frequency.from_f(freqs_hz, 'hz')
                two_port_network = tp.interpolate(interp_f) #, fill_value='extrapolate')
            step.ideal = two_port_network
        return steps

    def _new_steps_for_module_type(module_type):
        if module_type in ['TopShelf40', 'TopShelf70']:
            return _new_steps_36585()
        raise ValueError(f"Unknown Anritsu Autocal type '{module_type}'. Known types are TopShelf40 and TopShelf70.")
    
    def _new_steps_36585():
        steps = [
            AutocalStep(name='Short Short',   cmd='so1o2o3o4',  lname='Short.L', rname='Short.R'),
            AutocalStep(name='Open  Short',   cmd='so2o3o4',    lname='Open.L',  rname='Short.R'),
            AutocalStep(name='Short Open',    cmd='so1o2o4',    lname='Short.L', rname='Open.R'),
            AutocalStep(name='Load1 Short',   cmd='so3o4',      lname='Load1.L', rname='Short.R'),
            AutocalStep(name='Short Load1',   cmd='so1o4',      lname='Short.L', rname='Load1.R'),
            AutocalStep(name='Load2 Short',   cmd='so2o3',      lname='Load2.L', rname='Short.R'),
            AutocalStep(name='Short Load2',   cmd='so1o2',      lname='Short.L', rname='Load2.R'),
            AutocalStep(name='Load3 Short',   cmd='so3',        lname='Load3.L', rname='Short.R'),
            AutocalStep(name='Short Load3',   cmd='so1',        lname='Short.L', rname='Load3.R'),
            AutocalStep(name='Thru1',         cmd='so2o4',      tname='Thru1'),
            AutocalStep(name='Thru2',         cmd='so4',        tname='Thru2'),
            AutocalStep(name='Thru3',         cmd='so2',        tname='Thru3'),
            AutocalStep(name='Thru4',         cmd='s',          tname='Thru4')
        ]
        return steps
        
    def _get_cmplx_arr(self, name):
        '''
        Read a complex number array formatted as a big string of
        "real_part,im_part\n" lines.
        '''
        def cmplx_from_l(l):
            real_str, cmplx_str = l.split(',')
            r = float(real_str.strip())
            c = float(cmplx_str.strip())
            return r+c*(1j)
        matching_tags = self.acd.getElementsByTagName(name)
        assert len(matching_tags)==1
        return np.array([cmplx_from_l(l) for l in
                         matching_tags[0].childNodes[0].wholeText.split('\n')
                         if len(l)>0])

    def _two_port_from_4param(self, s11_name, s12_name, s21_name, s22_name):
        '''
        Read a S parameter matrix array formatted as 4 complex arrays,
        field_name.s11, field_name.s12, field_name.s21, field_name.s22.
        '''
        ntwk = Network()
        s11 = self._get_cmplx_arr(s11_name)
        s12 = self._get_cmplx_arr(s12_name)
        s21 = self._get_cmplx_arr(s21_name)
        s22 = self._get_cmplx_arr(s22_name)
        ntwk.s = np.array(\
                [[s11,s12],\
                 [s21,s22]]\
                ).transpose().reshape(-1,2,2)
        ntwk.frequency= skrf.Frequency.from_f(self.freqs_hz, 'Hz')
        return ntwk

    def _two_port_from_reflective(self, s11_name, s22_name):
        '''
        Read a S parameter matrix array from s11, s22, and the assumption
        that transmission components are zero.
        '''
        ntwk = Network()
        s11 = self._get_cmplx_arr(s11_name)
        s22 = self._get_cmplx_arr(s22_name)
        zzz = np.zeros_like(s11)
        ntwk.s = np.array(\
                [[s11,zzz],\
                 [zzz,s22]]\
                ).transpose().reshape(-1,2,2)
        ntwk.frequency= skrf.Frequency.from_f(self.freqs_hz, 'Hz')
        return ntwk
