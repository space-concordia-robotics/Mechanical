###IMPORT STATEMENTS###
import math
###CONSTANTS
g = 9.80655
###PIVOT MECHANISM###
###PIVOT - SLOPE ###
def setup_slope_matrix(variables, rover, theta):
    theta *= math.pi/180 #converts from degrees to radians
    (Fna, Fnb, Fnc, Frxa, Frxb, Frxc, Frya, Fryb, Fryc, T1, T2, T3, Fbx, Fby, mew) = variables
    return (mew*Fna - Frxa - rover.wheel_mass*g*math.sin(theta), \
            Fna - Frya - rover.wheel_mass*g*math.cos(theta), \
            mew*rover.wheel_radius*Fna - T1, \
            mew*Fnb - Frxb - rover.wheel_mass*g*math.sin(theta), \
            Fnb - Fryb - rover.wheel_mass*g*math.cos(theta), \
            mew*rover.wheel_radius*Fnb - T3, \
            mew*Fnc - Frxc - rover.wheel_mass*g*math.sin(theta), \
            Fnc - Fryc - rover.wheel_mass*g*math.cos(theta), \
            mew*rover.wheel_radius*Fnc - T3, \
            Frxa + Frxb - Fbx - rover.bogie_total_mass*g*math.sin(theta), \
            Fryb + Frya - Fby - rover.bogie_total_mass*g*math.cos(theta), \
            0.5*rover.bogie_length*(Frya - Fryb) + rover.wheel_link_length*(Frxa + Frxb) - T1 - T2, \
            Fbx + Frxc - rover.body_mass*g*math.sin(theta), \
            Fby + Fryc - rover.body_mass*g*math.cos(theta), \
            1.5*rover.bogie_length*Fryc - rover.wheel_link_length*Frxc - (rover.cg_length*math.cos(theta) + rover.cg_height*math.sin(theta))*rover.body_mass*g -T3\
            )
###PIVOT _ LOW OBSTACLE###
def setup_low_obs_matrix(variables, rover, h):
    alpha = 0
    beta = 0
    cangle = math.asin((rover.wheel_radius - h)/rover.wheel_radius)
    print(cangle)
    (Fna, Fnb, Fnc, Frxa, Frxb, Frxc, Frya, Fryb, Fryc, T1, T2, T3, Fbx, Fby, mew) = variables
    return (Fna*(-math.cos(cangle) + mew*math.sin(cangle)) + Frxa, \
            Fna*(math.sin(cangle) + mew*math.cos(cangle)) - Frya - rover.wheel_mass*g, \
            rover.wheel_radius*mew*Fna - T1, \
            mew*Fnb - Frxb, \
            Fnb - Fryb - rover.wheel_mass*g, \
            rover.wheel_radius*mew*Fnb - T2, \
            mew*Fnc - Frxc, \
            Fnc - Fryc - rover.wheel_mass*g, \
            rover.wheel_radius*mew*Fnc - T3, \
            -Frxa + Frxb + Fbx, \
            Frya + Fryb - Fby - rover.bogie_total_mass*g, \
            ( 0.5*rover.bogie_length*math.sin(alpha) - rover.wheel_link_length*math.cos(alpha))*Frxa + \
            ( 0.5*rover.bogie_length*math.cos(alpha) + rover.wheel_link_length*math.sin(alpha))*Frya + \
            (-0.5*rover.bogie_length*math.sin(alpha) + rover.wheel_link_length*math.cos(alpha))*Frxb + \
            (-0.5*rover.bogie_length*math.cos(alpha) + rover.wheel_link_length*math.sin(alpha))*Fryb + T1 + T2,  \
            2*Frxc - 2*Fbx, \
            2*Fryc + 2*Fby - rover.body_mass*g, \
            (-2*1.5*rover.bogie_length*math.sin(beta) - 2*rover.wheel_link_length*math.cos(beta))* -Frxc + \
            ( -2*1.5*rover.bogie_length*math.cos(beta) + 2*rover.wheel_link_length*math.sin(beta))*Fryc - \
            ( -rover.cg_length*math.cos(beta) - rover.cg_height*math.sin(beta)) * rover.body_mass*g +T3\
            )
###PIVOT - HIGH OBSTACLE###
def setup_high_obs_matrix(variables, rover, h):
    alpha = math.asin((h - rover.wheel_radius)/rover.bogie_length)
    beta = (rover.wheel_link_length*math.cos(alpha) + 0.5*rover.bogie_length*math.sin(alpha) - rover.wheel_link_length)/(1.5*rover.bogie_length)
    print("ALPHA:", alpha*180.0/math.pi)
    print("BETA:", beta*180.0/math.pi)
    (Fna, Fnb, Fnc, Frxa, Frxb, Frxc, Frya, Fryb, Fryc, T1, T2, T3, Fbx, Fby, mew) = variables
    return (-Fna + Frxa, \
            -Frya + mew*Fna - rover.wheel_mass*g, \
            rover.wheel_radius*mew*Fna - T1, \
            mew*Fnb - Frxb, \
            Fnb - Fryb - rover.wheel_mass*g, \
            rover.wheel_radius*mew*Fnb - T2, \
            mew*Fnc - Frxc, \
            Fnc - Fryc - rover.wheel_mass*g, \
            rover.wheel_radius*mew*Fnc - T3, \
            -Frxa + Frxb + Fbx, \
            Frya + Fryb - Fby - rover.bogie_total_mass*g, \
            ( 0.5*rover.bogie_length*math.sin(alpha) - rover.wheel_link_length*math.cos(alpha))*Frxa + \
            ( 0.5*rover.bogie_length*math.cos(alpha) + rover.wheel_link_length*math.sin(alpha))*Frya + \
            (-0.5*rover.bogie_length*math.sin(alpha) + rover.wheel_link_length*math.cos(alpha))*Frxb + \
            (-0.5*rover.bogie_length*math.cos(alpha) + rover.wheel_link_length*math.sin(alpha))*Fryb + T1 + T2,  \
            2*Frxc - 2*Fbx, \
            2*Fryc + 2*Fby - rover.body_mass*g, \
            (-2*1.5*rover.bogie_length*math.sin(beta) - 2*rover.wheel_link_length*math.cos(beta))* -Frxc + \
            ( -2*1.5*rover.bogie_length*math.cos(beta) + 2*rover.wheel_link_length*math.sin(beta))*Fryc - \
            ( -rover.cg_length*math.cos(beta) - rover.cg_height*math.sin(beta)) * rover.body_mass*g +T3\
            )
###4BAR###
###4 BAR - HIGH OBSTACLE###
def setup_4bar_high_matrix(variables, rover, h):
    alpha = math.asin((h - rover.wheel_radius)/rover.bogie_length)
    beta = (rover.wheel_link_length*math.cos(alpha) + 0.5*rover.bogie_length*math.sin(alpha) - rover.wheel_link_length)/(1.5*rover.bogie_length)
    print("ALPHA:", alpha*180.0/math.pi)
    print("BETA:", beta*180.0/math.pi)
    (Fna, Fnb, Fnc, Frxa, Frxb, Frxc, Frya, Fryb, Fryc, T1, T2, T3, Fl, Fr, mew) = variables
    return (-Fna + Frxa, \
            -Frya + mew*Fna - rover.wheel_mass*g, \
            rover.wheel_radius*mew*Fna - T1, \
            mew*Fnb - Frxb, \
            Fnb - Fryb - rover.wheel_mass*g, \
            rover.wheel_radius*mew*Fnb - T2, \
            mew*Fnc - Frxc, \
            Fnc - Fryc - rover.wheel_mass*g, \
            rover.wheel_radius*mew*Fnc - T3, \
            -Frxa + Frxb + Fl*math.sin(alpha - gamma) + Fr*math.sin(alpha + delta), \
            Frya + Fryb - Fl*math.cos(alpha - gamma) + Fr*math.cos(alpha + delta) - rover.bogie_total_mass*g, \
            0.5*db*(Fl*math.cos(gamma) - Fr*math.cos(delta)) - Frxa* ( rover.wheel_link_length*math.cos(alpha) - 0.5* rover.bogie_length * math.sin(alpha)) + \
            (0.5* rover.bogie_length * math.cos(alpha) + rover.wheel_link_length * math.sin(alpha)) * Frya - \
            (0.5* rover.bogie_length * math.cos(alpha) - rover.wheel_link_length * math.sin(alpha)) * Fryb + \
            (0.5* rover.bogie_length * math.sin(alpha) + rover.wheel_link_length * math.cos(alpha)), \
            2*Frxc + 2*Fl*math.sin(epsilon - beta) - 2*Fr*math.sin(phi + beta), \
            2*Fryc + 2*Fl*math.sin(epsilon - beta) + 2*Fr*math.sin(phi + beta) - rover.body_mass*g, \
            ((rover.wheel_link_length + rover.cg_height)*math.sin(beta)*rover.body_mass + \
            ((1.5*rover.bogie_length - 0.5*dc)*math.cos(epsilon) - rover.wheel_link_length*math.sin(epsilon)) *Fl + \
            ((1.5*rover.bogie_length + 0.5*dc)*math.cos(    phi) + rover.wheel_link_length*math.sin(    phi)) * Fr
            )
            )
