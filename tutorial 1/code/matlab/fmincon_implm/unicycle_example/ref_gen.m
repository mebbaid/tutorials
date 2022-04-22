function xr = ref_gen(tracking,const_ref, t)
    
    if tracking==0
        xr = const_ref;
    else
        xr = [t;t;pi/4];  % straightline
    end

end

