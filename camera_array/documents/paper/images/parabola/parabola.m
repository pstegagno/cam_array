clear all 
close all

 y_v= 1;
 y_p= 0.5;
 
 x_v = [0.85 0.675 0.56 0.40];
 x_p = [0.73 0.73 0.62 0.5];
 value = 0:0.01:1;
 
 area= {'vegetation','grass', 'soil','water'};

 color={ 'k', 'g', 'r', 'b'};
 
 for i=1:4
     
     par = zeros(1,size(value,2));
     
    if (i==2 || i==3)  
      a=(y_p-y_v)/((x_p(i)-x_v(i))*(x_p(i)-x_v(i)));
      c=y_v + a*x_v(i)*x_v(i);

      b=-2*a*x_v(i);

      for j=1:size(value,2)
           par(j) = a*value(j)*value(j)+b*value(j)+c;
      end 
  
    end
    
    if (i==1)
        
        par = ones(1,101);
        
         a=(y_p-y_v)/((x_p(i)-x_v(i))*(x_p(i)-x_v(i)));
         c=y_v + a*x_v(i)*x_v(i);
         b=-2*a*x_v(i);
         
         for j=1:85
           par(j) = a*value(j)*value(j)+b*value(j)+c;
         end
        
    end
    
     if (i==4)
        
        par = ones(1,101);
        
         a=(y_p-y_v)/((x_p(i)-x_v(i))*(x_p(i)-x_v(i)));
         c=y_v + a*x_v(i)*x_v(i);
         b=-2*a*x_v(i);
         
         for j=41:101
           par(j) = a*value(j)*value(j)+b*value(j)+c;
         end
     end
    
  legendInfo{i} = [area{i}]; 
    
  figure(1)
  plot((2*value-1),par,color{i})
 
  axis([-1 1 0 1.1])
  
  hold on 
  grid on
 
     
 end
 
 legend(legendInfo)
	  