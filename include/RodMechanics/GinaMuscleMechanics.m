classdef GinaMuscleMechanics < RodMechanicsBase

    properties
        % Actuator characterization bounds determined from the plot in
        % Olson 2021 IJSS.

    end

    methods
        function obj = GinaMuscleMechanics(l_0)
            arguments
                l_0 = 1;
            end
            f_force = @GinaMuscleMechanics.actuatorForce_key;
            obj@RodMechanicsBase(l_0, f_force);

            obj.e_bounds = [-0.5, 0.05];
            obj.a_bounds = [0, 100];
        end
    end

    methods(Static)
        % Gina Olson 2019
        % Copied into this class and reorganized by Bill Fan on 7/20/2023
        % Mechanics of Gina's McKibben muscles from the Olson 2020 IJSS
        % paper.
        function force = actuatorForce_key(strain,pressure)
        
        %e_min = -0.44;
        %e_max = 0.05
        
        e_crit = -0.303225592;
        e_lim = -0.274484489;
        
        kIa = 3.695687763;
        kIb = [0.637225654	0.453137516];
        kIIa = [1.038555666	0.511454549];
        kIIb = [1.963398204	-3.169979385	4.231857967];
        kIIIa = [265.2968218	320.5906703	126.11975	21.92761236];
        kIIIb = [89998.04265	836.5810641	62.59606786];
        kIV = [34610.2847	5991.799518	-1032.874471	-395.9792747];

        function f = pressComp_Ia(strain,pressure,e_crit,e_lim,kIV,kIIIa,kIb,kIa)            
            Pz = freeContract_IV(e_lim, kIV);
            
            compForce_limit = polyval([kIIIa,0],e_lim);
            critPressure = (compForce_limit/(kIb(1)*(kIb(2)+e_lim)^2))+Pz;
            
            compForce =  polyval([kIIIa,0],strain); 
            
            k_bc = (kIb(1)*(kIb(2)+e_lim)^2)/((-e_crit+e_lim)^2);
            
            if pressure<=critPressure
                f = compForce;
            else
                if -e_crit+strain>=0
                    f = compForce + k_bc*((-e_crit+strain)^2)*(pressure-critPressure);
                else
                    f = compForce - kIa*((-e_crit+strain)^2)*(pressure-critPressure);
                end
            end
        end
        
        function f = pressComp_Ib(strain,pressure,zfp,kIIIa, kIb)
            compForce = polyval([kIIIa,0],strain);
            critPressure = (compForce/(kIb(1)*(kIb(2)+strain)^2))+zfp;
            
            if pressure<=critPressure
                f = compForce;
            else
                f = compForce + kIb(1)*((kIb(2)+strain)^2)*(pressure-critPressure);
            end
        end
        
        function f = actuation_RIIa(strain,pressure, zfp, kIIa)
            f = kIIa(1)*((kIIa(2)+strain)^2)*(pressure-zfp);
        end
        
        function f = pressureExt_RIIb(strain,pressure,kIIIb,kIIa,kIIb)
            k_bc = sqrt(kIIa(1)*(kIIa(2)^2)/kIIb(1));
            pE = polyval([kIIIb,0],strain);
            f = pE + kIIb(1)*((k_bc+kIIb(2)*strain)^2)*(pressure^(1+kIIb(3)*strain)); 
        end
        
        %Region IIIa passive compression
        %fit coefficients do not automatically include +C coeff
        function f = passiveComp_RIIIa(strain, kIIIa)
            f = polyval([kIIIa,0],strain);
        end
        
        %Region IIIb passive extension
        %fit coefficients do not automatically include +C coeff
        function f = passiveExt_RIIIb(strain, kIIIb)
            f = polyval([kIIIb,0],strain);
        end
        
        function p = freeContract_IV(strain, kIV)
            p = polyval([kIV, 0], strain);
        end
        
        if pressure==0
            if strain>=0 %passive extension
                force = passiveExt_RIIIb(strain, kIIIb);
            else %passive compression
                force = passiveComp_RIIIa(strain, kIIIa);
            end
        else
            if strain>=0 %pressurized extension
                force = pressureExt_RIIb(strain,pressure,kIIIb,kIIa,kIIb);
            else %active region or pressurized compression                
                zfp = freeContract_IV(strain, kIV);

                if pressure>=zfp %active region
                    force = actuation_RIIa(strain,pressure, zfp, kIIa);
                else
                    if strain>=e_lim %pressurized comp Ib
                        force = pressComp_Ib(strain,pressure,zfp,kIIIa, kIb);
                    else %pressurized comp Ia
                        force = pressComp_Ia(strain,pressure,e_crit,e_lim,kIV,kIIIa,kIb,kIa);
                    end
                end
            end
        end
        
        force = -force;
        end
    end
end

