import numpy as np


class QuadcopterPhysics:
    def __init__(self, mass, arm_length, k_l, k_d):
        self.mass = mass
        self.L = arm_length
        self.k_l = k_l  # lift_constant
        self.k_d = k_d  # drag_constant
        self.g = 9.81

    def compute_motor_speeds(self, thrust_cmd, tau_phi, tau_theta, tau_psi):
        """
        Resuelve la dinámica inversa para obtener w1, w2, w3, w4.
        Basado en la configuración en X del Mavic 2 Pro.
        """
        # 1. Empuje total necesario para compensar gravedad + comando
        # El comando 'thrust_cmd' actúa sobre la aceleración vertical
        T = (self.mass * self.g) + thrust_cmd

        # 2. Resolver sistema para w^2 (Velocidades al cuadrado)
        # Estas fórmulas se obtienen despejando el modelo de MATLAB
        w1_sq = T/(4*self.k_l) - tau_theta / \
            (2*self.k_l*self.L) - tau_psi/(4*self.k_d)
        w2_sq = T/(4*self.k_l) - tau_phi / \
            (2*self.k_l*self.L) + tau_psi/(4*self.k_d)
        w3_sq = T/(4*self.k_l) + tau_theta / \
            (2*self.k_l*self.L) - tau_psi/(4*self.k_d)
        w4_sq = T/(4*self.k_l) + tau_phi / \
            (2*self.k_l*self.L) + tau_psi/(4*self.k_d)

        # 3. Aplicar raíz cuadrada y asegurar que no haya valores negativos
        return [
            np.sqrt(max(0, w1_sq)),
            np.sqrt(max(0, w2_sq)),
            np.sqrt(max(0, w3_sq)),
            np.sqrt(max(0, w4_sq))
        ]
