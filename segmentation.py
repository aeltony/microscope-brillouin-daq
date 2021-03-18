import numpy as np
import matplotlib.pyplot as plt
from lmfit import Model
from timeit import default_timer as timer
from scipy.ndimage.filters import gaussian_filter

#f = open('/Users/ame38/Desktop/Sample-Alines/Aline-P1-1.txt','r')
#dat = np.loadtxt(f)
#f.close()
dat = np.genfromtxt('/Users/ame38/Desktop/Sample-Alines/Aline-P11-28.txt', delimiter=",", filling_values=np.nan)
aline = dat[:,0]
signal = dat[:,1]
#print 'aline =', aline
#print 'signal=', signal

start = timer()

step_size = 30. #microns
dist = step_size * np.arange(len(aline))
weights = np.sqrt(signal)

plt.figure()
plt.subplot(3,1,1)
plt.plot(dist, aline, '.k')

plt.subplot(3,1,3)
plt.plot(dist, signal, 'k--')
plt.xlim(0,1200)

sigThresh = 0.4*np.nanmean(signal[signal>100])

cleanIdx = (~np.isnan(aline)) & (signal > sigThresh) & (aline > 5) & (aline < 6)
signal = signal[cleanIdx]
dist = dist[cleanIdx]
weights = weights[cleanIdx]
aline = aline[cleanIdx]

blurred = gaussian_filter(aline, sigma=1)

deriv = np.gradient(blurred,dist)
edgeIdx = np.argmin(deriv)
plt.subplot(3,1,1)
plt.axvline(x=dist[edgeIdx],color='g')

## Find signal spike corresponding to cornea surface
#idx = int(np.floor(0.5*len(signal)))
#idxStart = np.nanargmax(signal[:idx]) - 1
#startPos = dist[idxStart]
#

sortAline = np.argsort(blurred)
stromaGuess = np.mean(blurred[sortAline[-4:]])
aqGuess = np.mean(blurred[sortAline[:4]])
print 'stromaGuess =', stromaGuess
print 'aqGuess =', aqGuess
widthGuess = (aqGuess - stromaGuess)/deriv[edgeIdx]
endGuess = dist[edgeIdx]-0.5*widthGuess

def alineShape(x, endStroma, stromaBS, width, aqBS):
    condlist = [ x < endStroma,
                (x >= endStroma) & (x < endStroma + width),
                x >= endStroma + width
                ]
    funclist = [lambda x: stromaBS,
                lambda x: x*(aqBS-stromaBS)/width + stromaBS - endStroma*(aqBS-stromaBS)/width,
                lambda x: aqBS
                ]
    return np.piecewise( x, condlist, funclist )

model = Model(alineShape)
pars = model.make_params()
pars['endStroma'].set(endGuess, min=dist[0], max=dist[-1])
pars['stromaBS'].set(stromaGuess, min=5.4, max=6.)
pars['width'].set(widthGuess, min=30., max=1000)
pars['aqBS'].set(aqGuess, min=5.0, max=5.4)

result = model.fit(aline, pars, x=dist, weights=weights, method='Nelder')

idxEnd = (np.abs(dist - result.params['endStroma'].value)).argmin()
strIdx = (dist < dist[idxEnd]) & (abs(aline - np.mean(aline[:idxEnd])) < 0.15) #1*np.std(aline[:idxEnd]))

end = timer()
print 'time:', end-start

plt.subplot(3,1,1)
plt.axvline(x=dist[idxEnd],color='b')
plt.axvline(x=dist[0],color='b')

#print(result.fit_report())
plt.subplot(3,1,1)
plt.plot(dist, blurred, '--y')
plt.ylim(5,6)
plt.xlim(0,1200)

#plt.plot(dist[strIdx],aline[strIdx],'ob')

xx = np.linspace(dist[0],dist[-1],len(dist)*4)
plt.plot(xx,model.eval(pars,x=xx),'--g')
plt.plot(xx, model.eval(result.params,x=xx),'b')

xdat = dist[:idxEnd]
ydat = result.params['stromaBS'].value * np.ones(dist[:idxEnd].shape)
plt.plot(xdat,ydat,'r')

plt.subplot(3,1,2)
plt.plot(dist, deriv, 'b')
plt.xlim(0,1200)

plt.show()


