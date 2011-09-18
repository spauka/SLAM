Constants
chi_t = GenerateInitialDistribution();
while 1
   [chi_t, chi_t_bar] = ParticleFilter(chi_t, [0;0], [0;0;10;10;10;10]);
   scatter(chi_t(1,:), chi_t(2,:));
   pause
end