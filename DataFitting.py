# By Amira, 02/2019, for fitting spectral data (Lorentzians)
# Updated 02/2021
import numpy as np
import time
from scipy.optimize import curve_fit
from scipy.signal import find_peaks
from timeit import default_timer as timer
from scipy.ndimage.filters import gaussian_filter

#### Fit Brillouin spectrum,
# sline is the data (counts) for the pixels on the spectral line,
# ftol and xtol are fitting tolerances (adjust for speed vs. accuracy)
def fitSpectrum(sline, xtol=1e-6, ftol=1e-6, maxfev=500):
	start = timer()
	posSline = np.abs(sline)
	weights = np.sqrt(posSline)  # Weight data by SNR

	# Find peak locations:
	prominence = 0.05*np.amax(posSline)
	wlen = 5*prominence
	pk_ind, pk_info = find_peaks(sline, prominence=prominence, width=2, \
		height=100, rel_height=0.5, wlen=wlen)
	pk_wids = 0.5*pk_info['widths']
	pk_hts = np.pi*pk_wids*pk_info['peak_heights']
	#print("pk_ind = ", pk_ind)
	
	# Check for extra peaks from adjacent orders:
	if len(pk_ind)>2:
		pk_srt = np.argsort(pk_hts)
		pk_ind = pk_ind[pk_srt[-2:]]
		pk_wids = pk_wids[pk_srt[-2:]]
		pk_hts = pk_hts[pk_srt[-2:]]
	
	# Check for overlapping peaks:
	if len(pk_ind)==1:
		pk_ind = np.array([pk_ind[0]-2, pk_ind[0]+2])
		pk_wids = np.array([pk_wids[0]-2, pk_wids[0]+2])
		pk_hts = np.array([pk_hts[0]-2, pk_hts[0]+2])

	# Check for no peaks:
	if len(pk_ind)<1:
		#print('[AndorDevice] Too few peaks in spectrum')
		interPeaksteps = np.array([])
		fittedSpect = np.nan*np.ones(sline.shape)
		return (interPeaksteps, fittedSpect)

	# Starting guesses for fit:
	p0 = [pk_hts[0], pk_ind[0], pk_wids[0], pk_hts[1], pk_ind[1], pk_wids[1], np.amin(sline)]
	
	pix = np.arange(0, sline.shape[0]) # Pixel number
	
	# Create boolean mask to filter out points far from the peaks:
	pk_mask = np.array(0*sline, dtype=bool)
	#win_fac = 5
	#for n in np.arange(0,4):
	#	pk_mask[(pk_ind[n] - win_fac*pk_wids[n]).astype(int):(pk_ind[n] + win_fac*pk_wids[n]).astype(int)]=True
	pk_mask[:]=True # Do not use mask

	# Fit spectrum to 2-Lorentzian model:
	try:
		popt, pcov = curve_fit(_2Lorentzian, pix[pk_mask], sline[pk_mask], p0=p0, ftol=ftol, xtol=xtol)
		end = timer()
		#residuals = sline[pk_mask] - _4Lorentzian(pix[pk_mask], \
		#	popt[0], popt[1], popt[2], popt[3], popt[4], popt[5], popt[6], \
		#	popt[7], popt[8], popt[9], popt[10], popt[11], popt[12])
		#ss_res = np.sum(residuals**2)
		#ss_tot = np.sum((sline[pk_mask]-np.mean(sline[pk_mask]))**2)
		#r_squared = 1 - (ss_res / ss_tot)
		#print('Spectrum fitting time =', (end-start)*1e3, 'ms')
		#print('R^2 =', r_squared)
		#perr = np.sqrt(np.diag(pcov))
		interPeaksteps = np.zeros(2)
		interPeaksteps[0] = np.array([np.average(pk_info['peak_heights'])])
		interPeaksteps[1] = np.abs(popt[4] - popt[1])
		fittedSpect = _2Lorentzian(pix, popt[0], popt[1], popt[2], popt[3], popt[4], popt[5], popt[6])
	except:
		#print('[AndorDevice] Fitting spectrum failed')
		interPeaksteps = np.array([])
		fittedSpect = np.nan*np.ones(sline.shape)

	return (interPeaksteps, fittedSpect)

#### Fit calibration curve to determine SD and FSR
def fitCalCurve(pxDist, freq, xtol=1e-6, ftol=1e-6, maxfev=500):
	start = timer()
	# Starting guesses for fit:
	p0 = [0.127, 21.5]
	try:
		popt, pcov = curve_fit(_Linear, pxDist, freq, p0=p0, ftol=ftol, xtol=xtol)
		end = timer()
		residuals = freq - _Linear(pxDist, popt[0], popt[1])
		ss_res = np.sum(residuals**2)
		ss_tot = np.sum((freq-np.mean(freq))**2)
		r_squared = 1 - (ss_res / ss_tot)
		print('Calibration curve fitting time =', (end-start)*1e3, 'ms')
		print('R^2 =', r_squared)
		#perr = np.sqrt(np.diag(pcov))
		SD = popt[0]
		FSR = popt[1]
	except:
		SD = np.nan
		FSR = np.nan
	return (SD, FSR)

def _2Lorentzian(x, amp1,cen1,wid1, amp2,cen2,wid2, offs):
    return (amp1*wid1**2/((x-cen1)**2+wid1**2)) \
            + (amp2*wid2**2/((x-cen2)**2+wid2**2)) \
            + offs

def _4Lorentzian(x, amp1,cen1,wid1, amp2,cen2,wid2, amp3,cen3,wid3, amp4,cen4,wid4, offs):
    return (amp1*wid1**2/((x-cen1)**2+wid1**2)) \
            + (amp2*wid2**2/((x-cen2)**2+wid2**2)) \
            + (amp3*wid3**2/((x-cen3)**2+wid3**2)) \
            + (amp4*wid4**2/((x-cen4)**2+wid4**2)) \
            + offs

def _Linear(x, sd, fsr):
	return 0.5*fsr - 0.5*sd*x