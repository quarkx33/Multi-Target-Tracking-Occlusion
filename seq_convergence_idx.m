function r = seq_convergence_idx(data, thresh)

[rows,len] = size(data);
tmp_convergence_id = Inf(rows,1);

%old fashioned way of finding the earliuest convergence
  for ii = 1: rows
      c=1;
      
      while(c<=(len-thresh+1))
        start_seq = find(data(ii,c:end)==1,1,'first');
        if isempty(start_seq)
            break;
        end
        c = c+start_seq-1;
        
        end_seq = find(data(ii,c:end)==0,1,'first');
        if isempty(end_seq)
            if (len-(c-1))>=thresh
                tmp_convergence_id(ii) = c;
            else
                c=Inf;
                break;
            end
        end
        
        if (end_seq-1)>=thresh
            tmp_convergence_id(ii) = c;
        else
            c = c+end_seq-1;
        end
      
      end
      
      
  end
  
  
  
  convergence_idx = min(tmp_convergence_id);
  if isinf(convergence_idx)
      r = len;
  else
      r = convergence_idx;
  end
  
  