from scipy.fftpack import fft
import matplotlib.pyplot as plt


N = 600

T = 1.0/800.0
x = np.linspace(0.0, N*T, N)
y = np.sin(50.0*2.0*np.pi*x) + 0.5*np.sin(80.0 *2.0*np.pi*x)

yf = fft(y)
xf = np.linspace(0.0, 1.0/(2.0*T), N//2)
