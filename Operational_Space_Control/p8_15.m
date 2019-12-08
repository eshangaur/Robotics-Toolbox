if c==0,
% joint positions
  subplot(2,2,1);
  plot(time, x_d(1)*ones(size(time)),'--',time,x(:,1));
  grid on
  axis([0 t_d -2.5 2.5])
%  set(gca,'fontname','Times','fontsize',12,'fontweight','normal');
  xlabel('[s]');
  ylabel('[m]');
  title('end-effector x-pos');

  subplot(2,2,2);
  plot(time, x_d(2)*ones(size(time)),'--',time,x(:,2));
  grid on
  axis([0 t_d -2.5 2.5])
%  set(gca,'fontname','Times','fontsize',12,'fontweight','normal');
  xlabel('[s]');
  ylabel('[m]');
  title('end-effector y-pos');

else

% joint positions
  subplot(2,2,1);
  plot(time, x_d(1)*ones(size(time)),'--',time,x(:,1));
  grid on
  axis([0 t_d -2.5 2.5])
%  set(gca,'fontname','Times','fontsize',12,'fontweight','normal');
  xlabel('[s]');
  ylabel('[m]');
  title('end-effector x-pos');

  subplot(2,2,2);
  plot(time, x_d(2)*ones(size(time)),'--',time,x(:,2));
  grid on
  axis([0 t_d -2.5 2.5])
%  set(gca,'fontname','Times','fontsize',12,'fontweight','normal');
  xlabel('[s]');
  ylabel('[m]');
  title('end-effector y-pos');

end;
