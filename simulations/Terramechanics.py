###IMPORT STATEMENTS###
import math
def compute_solid_wheel_contact_length(rover, soil, W):
    Ce = (1-math.pow(rover.poisson,2))/rover.youngs_modulus \
     + (1-math.pow(soil.poisson,2))/soil.youngs_modulus
    return 1.60*math.sqrt(W * 2 * rover.wheel_radius * Ce / rover.wheel_width)

def compute_contact_area(rover, soil, W):
    if rover.solid_wheel is True:
        l_con = compute_solid_wheel_contact_length(rover, soil, W)
        return l_con * rover.wheel_width
    else:
        print("ERROR: FLEXIBLE WHEEL MODULE NOT IMPLEMENTED")
        return 0

def compute_wheel_thrust(c, phi, Aw, W, slope_angle, J_K = 3):
    return (c*Aw + W*math.cos(slope_angle*math.pi/180.0)*math.tan(phi*math.pi/180.0))*(1.0-math.exp(-J_K))

def compute_compaction_resistance(b, kc, kphi, l,  z, n):
    if(b > l):
        return b*(kc/l + kphi)*math.pow(z,n+1)/(n+1)
    else:
        return b*(kc/b + kphi)*math.pow(z,n+1)/(n+1)

def compute_bulldozing_resistance(d, b, z, phi, c, W, rho, g):
    phi *= math.pi/180
    alpha = math.acos(1 - 2*z/d)
    sw = rho* g
    l_p = z*math.pow(math.tan(0.25*math.pi - phi/2),2)
    NC = W/(2*l_p*b*c)
    phi_n = math.atan(2.0*math.tan(phi)/3.0)
    kc = (NC - math.tan(phi_n))*math.pow(math.cos(phi_n),2)
    N_sw = W/(2 * b * sw)
    k_sw = (2*N_sw/math.tan(phi_n) + 1)*math.pow(math.cos(phi_n),2)
    return b*math.sin(alpha + phi)*(2*z*c*kc/1000 + sw*z*z*k_sw)/(2*math.sin(alpha)*math.cos(phi)) \
    + (math.pi*sw*math.pow(l_p,3)*(90 - phi*180/math.pi)/540 + math.pi*c*l_p*l_p/180 + c*l_p*l_p*math.tan(0.25*math.pi + phi/2))

def compute_gravitational_resistance(W, slope_angle):
    slope_angle*math.pi/180
    return W*math.sin(slope_angle)

def compute_rolling_resistance(W, fr):
    return fr*W

def compute_total_resistance(d, b, z, W, l, slope_angle, fr, c, n, phi, kc, kphi, rho, g):
    Rc = compute_compaction_resistance(b, kc, kphi, l, z, n)
    Rb = compute_bulldozing_resistance(d, b, z, phi, c, W, rho, g)
    Rg = compute_gravitational_resistance(W, slope_angle)
    Rf = compute_rolling_resistance(W, fr)
    return Rc + Rb + Rg + Rf

def compute_drawbar_pull(d, b, z, W, l, Aw, slope_angle, fr, c, n, phi, kc, kphi, rho, g, J_K = 3):
    H = compute_wheel_thrust(c, phi, Aw, W, slope_angle, J_K)
    R = compute_total_resistance(d, b, z, W, l, slope_angle, fr, c, n, phi, kc, kphi, rho, g)
    return H - R

###CALCULATION MATRIX SETUP###
def mew_approximation(rover, soil):
    if rover.solid_wheel is True:
        W = rover.total_mass*g/3
        Aw = compute_contact_area(rover, soil, W)
        Te = compute_wheel_thrust(soil.c, soil.phi, Aw, W, 0)
        return Te/W
