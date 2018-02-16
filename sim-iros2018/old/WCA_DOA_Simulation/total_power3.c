
/*
 
  Compute Ray path with 3 reflexions
 
  Usage
  ------
 
  rs_amp = total_power3(TXpoint , RXpoint , planes , material , fc  , [nr] , [flp_scale]);
 
 
  Inputs
  -------
 
  TXpoint                               Transmitter point (3 x Nt)
  RXpoint                               Receiver point (3 x Nr)
  planes                                Planes (22 x nplanes)
  material                              Material (6 x nplanes)
  fc                                    Central Frequency
  nr                                    Numbre of reflexion (nr>= 0 and nr < 4, default nr = 3)
  flp_scale                             Scale factor (default = 100)
 
  Outputs
  -------
 
  rs_amp                                Total power (Nt x Nr)
 
 
  To compile
  ----------
 
 
  mex    -output total_power3.dll total_power3.c
 
  mex    -f mexopts_intel10amd.bat -output total_power3.dll total_power3.c
 
 
 
  Example 1
  ---------
 
  flp                      = load_flp('norwich01.flp');
  nr                       = 1;
  Nt                       = 5;
  Nr                       = 10;
  temp                     = flp.geom.planes([1 , 4 , 7] , :);
  xmin                     = min(temp(:));
  xmax                     = max(temp(:));
  temp                     = flp.geom.planes([2 , 5 , 8] , :);
  ymin                     = min(temp(:));
  ymax                     = max(temp(:));
  temp                     = flp.geom.planes([3 , 6 , 9] , :);
  zmin                     = min(temp(:));
  zmax                     = max(temp(:));
  Cov                      = [1000000 0 0 ; 0 1000000 0 ; 0 0 100];
  flp.info.TXpoint         = flp.info.TXpoint(: , ones(1 , Nt)) + chol(Cov)'*randn(3 , Nt);
  flp.info.RXpoint         = flp.info.RXpoint(: , ones(1 , Nr)) + chol(Cov)'*randn(3 , Nr);
  rs_amp                   = total_power3(flp.info.TXpoint ,flp.info.RXpoint , flp.geom.planes , flp.geom.material , flp.info.fc , nr);
  plot_flp(flp);
 
 
 Author : Sébastien PARIS : sebastien.paris@lsis.org
 -------  Date : 10/09/2007
 
 Reference : Jacques Beneat, http://www2.norwich.edu/jbeneat/
 ---------
 
 */

#include <math.h>
#include <mex.h>

#define PI 3.14159265358979323846



void fct_calimage(double * , double * , int  , double *) ;
bool fct_calrpt(double *, double *, double * , int  , double * );
double * fct_caltpts(double *, double *, double *, int , double * , int *, double *);
void compute_distloss(double * , int , double * , double * , double  , double , int , double * , double *);
void qsindex( double * , int * , int , int  );





/*-------------------------------------------------------------------------------------------------------------- */


double c = 3e8 , Ant = 1.0 , loss_factor = 1.0 , t_tol = 1.0;


void mexFunction( int nlhs, mxArray *plhs[] , int nrhs, const mxArray *prhs[] )

{
    
    
    double *TXpoint , *RXpoint , *planes , *material;
    
    double fc , flp_scale;
    
    int    nr , Nt , Nr;
    
    double *rs_amp;
    
    
    
    
    double *rpath1=NULL , *rpath2=NULL , *rpath3=NULL;
    
    double *path0=NULL , *path1=NULL , *path2=NULL , *path3=NULL;
    
    double temp;
    
    
    int nplanes , i , j , k , l , n , id , jd , kd , indice , ind , lNr , nNt , ld;
    
    int npath1 , npath2 , npath3 , npath , currentpath  , nb_pts;
    
    
    
    double *TX , *RX , *TX_i , *Phiti , *TX_j , *Phitj , *Phitij , *TX_k , *Phitk , *Phitjk , *Phitijk , *Phit_nt;
    
    double *Thits0=NULL ;
    
    double *Thits11=NULL , *Thits12=NULL;
    
    double *Thits21=NULL , *Thits22=NULL, *Thits23=NULL;
    
    double *Thits31=NULL , *Thits32=NULL, *Thits33=NULL, *Thits34=NULL;
    
    double *distance , *lossfac;
    
    int tflag0 , tflag11 , tflag12 , tflag21 , tflag22 , tflag23 , tflag31 , tflag32 , tflag33 , tflag34;
    
    bool rflagi=false , rflagj=false , rflagij=false , rflagk=false , rflagjk=false , rflagijk=false;
    
    
    
    /* Inputs  */
    
    if( (nrhs <5) || (nrhs >7) )
        
    {
        
        mexErrMsgTxt("At least 5 inputs are requiered for total_power");
        
    }
    
    
    TXpoint    = mxGetPr(prhs[0]);
    
    if( mxGetNumberOfDimensions(prhs[0]) !=2 || mxGetM(prhs[0]) != 3 )
    {
        
        mexErrMsgTxt("TXpoint must be (3 x Nt)");
        
    }
    
    Nt         = mxGetN(prhs[0]);
    
    
    
    RXpoint    = mxGetPr(prhs[1]);
    
    if( mxGetNumberOfDimensions(prhs[1]) !=2 || mxGetM(prhs[1]) != 3 )
    {
        
        mexErrMsgTxt("RXpoint must be (3 x Nr)");
        
    }
    
    Nr         = mxGetN(prhs[1]);
    
    
    
    
    
    planes     = mxGetPr(prhs[2]);
    
    if( mxGetNumberOfDimensions(prhs[2]) !=2 || mxGetM(prhs[2]) !=22 )
    {
        
        mexErrMsgTxt("planes must be (22 x nplanes)");
        
    }
    
    
    
    nplanes     = mxGetN(prhs[2]);
    
    
    material    = mxGetPr(prhs[3]);
    
    if( mxGetNumberOfDimensions(prhs[3]) !=2 || mxGetM(prhs[3]) !=6 )
    {
        
        mexErrMsgTxt("material must be (6 x nplanes)");
        
    }
    
    
    
    
    fc          = mxGetScalar(prhs[4]);
    
    
    if( nrhs > 5)
        
    {
        nr          = (int)mxGetScalar(prhs[5]);
        
        if((nr < 0) || (nr > 3))
        {
            
            
            mexErrMsgTxt("nr must be [0,1,2,3]");
            
        }
        
    }
    
    else
    {
        
        nr = 3;
        
    }
    
    
    if( nrhs > 6)
        
    {
        
        flp_scale = mxGetScalar(prhs[6]);
        
    }
    
    else
        
    {
        
        flp_scale = 100.0;
        
    }
    
    
    /* Memory allocation  */
    
    
    TX                    = mxMalloc(3*sizeof(double));

    RX                    = mxMalloc(3*sizeof(double));

    
    TX_i                  = mxMalloc(3*sizeof(double));
    
    Phiti                 = mxMalloc(3*sizeof(double));
    
    TX_j                  = mxMalloc(3*sizeof(double));
    
    Phitj                 = mxMalloc(3*sizeof(double));
    
    Phitij                = mxMalloc(3*sizeof(double));
    
    TX_k                  = mxMalloc(3*sizeof(double));
    
    Phitk                 = mxMalloc(3*sizeof(double));
    
    Phitjk                = mxMalloc(3*sizeof(double));
    
    Phitijk               = mxMalloc(3*sizeof(double));
    
    Phit_nt               = mxMalloc(3*sizeof(double));
    
    


    /* Outputs  */
            
            
            
	plhs[0]               = mxCreateDoubleMatrix(Nt , Nr, mxREAL);
                     
	rs_amp                = mxGetPr(plhs[0]);


    
    for(l = 0 ; l < Nr ; l++)
    {
        
        lNr = l*3;

		ld  = l*Nt;

		for (i = 0 ; i < 3 ; i++)

		{

			RX[i]   = RXpoint[i + lNr];

		}



        for (n = 0 ; n < Nt ; n++)
        {
           
			nNt = n*3;
			
			for (i = 0 ; i < 3 ; i++)
				
			{
								
				TX[i]   = TXpoint[i + nNt];
				
			}
			
            
            npath1  = npath2 = npath3 = 0;
            
            currentpath = 0;
            
            if(nr > 0)
            {
                
                for (i = 0 ; i < nplanes ; i++)
                    
                {
                    
                    fct_calimage(TX , planes , i , TX_i);
                    
                    rflagi = fct_calrpt(TX_i, RX, planes , i , Phiti);
                    
                    if (rflagi == true)
                    {
                        
                        id             = npath1*5;
                        
                        npath1++;
                        
                        rpath1         = (double *)mxRealloc((double *)rpath1 , (npath1)*5*sizeof(double));
                        
                        rpath1[0 + id] = Phiti[0];
                        
                        rpath1[1 + id] = Phiti[1];
                        
                        rpath1[2 + id] = Phiti[2];
                        
                        rpath1[3 + id] = i;
                        
                        rpath1[4 + id] = 1.0;
                        
                    }
                    
                    if(nr > 1)
                    {
                        
                        for(j = 0 ; j < nplanes ; j++)
                            
                        {
                            if (j != i)
                            {
                                
                                fct_calimage(TX_i , planes , j , TX_j);
                                
                                rflagj = fct_calrpt(TX_j, RX, planes , j , Phitj);
                                
                                if (rflagj == true)
                                {
                                    
                                    rflagij = fct_calrpt(TX_i,  Phitj, planes , i , Phitij);
                                    
                                    if(rflagij == true)
                                    {
                                        jd             = npath2*10;
                                        
                                        npath2++;
                                        
                                        rpath2         = (double *)mxRealloc((double *)rpath2 , (npath2)*10*sizeof(double));
                                        
                                        
                                        rpath2[0 + jd] = Phitij[0];
                                        
                                        rpath2[1 + jd] = Phitij[1];
                                        
                                        rpath2[2 + jd] = Phitij[2];
                                        
                                        rpath2[3 + jd] = i;
                                        
                                        rpath2[4 + jd] = 1.0;
                                        
                                        rpath2[5 + jd] = Phitj[0];
                                        
                                        rpath2[6 + jd] = Phitj[1];
                                        
                                        rpath2[7 + jd] = Phitj[2];
                                        
                                        rpath2[8 + jd] = j;
                                        
                                        rpath2[9 + jd] = 1.0;
                                        
                                    }
                                    
                                }
                                if(nr > 2)
                                {
                                    for (k = 0 ; k < nplanes ; k++)
                                    {
                                        if(k != j)
                                        {
                                            
                                            fct_calimage(TX_j , planes , k , TX_k);
                                            
                                            rflagk = fct_calrpt(TX_k, RX, planes , k , Phitk);
                                            
                                            if(rflagk == true)
                                            {
                                                
                                                rflagjk = fct_calrpt(TX_j, Phitk, planes , j , Phitjk);
                                                
                                                if(rflagjk == true)
                                                {
                                                    
                                                    rflagijk = fct_calrpt(TX_i, Phitjk, planes , i , Phitijk);
                                                    
                                                    if(rflagijk == true)
                                                    {
                                                        kd              = npath3*15;
                                                        
                                                        npath3++;
                                                        
                                                        rpath3          = (double *)mxRealloc((double *)rpath3 , (npath3)*15*sizeof(double));
                                                        
                                                        
                                                        rpath3[0 + kd]  = Phitijk[0];
                                                        
                                                        rpath3[1 + kd]  = Phitijk[1];
                                                        
                                                        rpath3[2 + kd]  = Phitijk[2];
                                                        
                                                        rpath3[3 + kd]  = i;
                                                        
                                                        rpath3[4 + kd]  = 1.0;
                                                        
                                                        rpath3[5 + kd]  = Phitjk[0];
                                                        
                                                        rpath3[6 + kd]  = Phitjk[1];
                                                        
                                                        rpath3[7 + kd]  = Phitjk[2];
                                                        
                                                        rpath3[8 + kd]  = j;
                                                        
                                                        rpath3[9 + kd]  = 1.0;
                                                        
                                                        rpath3[10 + kd] = Phitk[0];
                                                        
                                                        rpath3[11 + kd] = Phitk[1];
                                                        
                                                        rpath3[12 + kd] = Phitk[2];
                                                        
                                                        rpath3[13 + kd] = k;
                                                        
                                                        rpath3[14 + kd] = 1.0;
                                                        
                                                    }
                                                    
                                                }
                                                
                                            }
                                            
                                        }
                                        
                                    }
                                }
                            }
                            
                        }
                        
                    }
                }
                
            }
            
            npath                  = npath1 + npath2 + npath3 + 1;
                        
            distance               = mxMalloc(npath*sizeof(double));
            
            lossfac                = mxMalloc(npath*sizeof(double));
                        
            
            // Direct path (0 reflexion)
            
            
            Thits0                 = fct_caltpts(TX , RX, planes , nplanes , material , &tflag0 , Phit_nt);
                        
            nb_pts                 = (2 + tflag0);
            
            path0                  = mxMalloc(nb_pts*5*sizeof(double));
            
            
            path0[0]               = TX[0];
            
            path0[1]               = TX[1];
            
            path0[2]               = TX[2];
            
            path0[3]               = 0.0;
            
            path0[4]               = 0.0;
            
            
            
            indice                  = (tflag0 + 1)*5;
            
            
            for(i = 5 ; i < indice; i++)
                
            {
                
                path0[i] = Thits0[i - 5];
                
            }
            
            
            path0[0 + indice]       = RX[0];
            
            path0[1 + indice]       = RX[1];
            
            path0[2 + indice]       = RX[2];
            
            path0[3 + indice]       = 0.0;
            
            path0[4 + indice]       = 0.0;
            
            
            compute_distloss(path0 , nb_pts , planes , material , fc , flp_scale  , currentpath , distance , lossfac);
            
            
            
            if(Thits0 != NULL)
            {
                
                free(Thits0);
                
                Thits0 = NULL;
                
            }
            
            if(path0 != NULL)
            {
                
                mxFree(path0);
                
                path0 = NULL;
                
            }
            
            
            
            
            
            // Path with 1 reflexion
            
            
            for (i = 0 ; i < npath1 ; i++)
                
            {
                
                currentpath++;
                
                id                     = i*5;
                
                
                Phiti[0]               = rpath1[0 + id];
                
                Phiti[1]               = rpath1[1 + id];
                
                Phiti[2]               = rpath1[2 + id];
                
                
                Thits11                = fct_caltpts(TX , Phiti, planes , nplanes , material , &tflag11 , Phit_nt);
                
                Thits12                = fct_caltpts(Phiti, RX, planes , nplanes , material , &tflag12 , Phit_nt);
                
                
                nb_pts                 = (tflag11+tflag12+3);
                
                path1                  = mxMalloc(nb_pts*5*sizeof(double));
                
                
                path1[0]               = TX[0];
                
                path1[1]               = TX[1];
                
                path1[2]               = TX[2];
                
                path1[3]               = 0.0;
                
                path1[4]               = 0.0;
                
                
                indice                  = (tflag11+1)*5;
                
                
                for(j = 5 ; j < indice ; j++)
                    
                {
                    
                    path1[j] = Thits11[j - 5];
                    
                }
                
                
                
                path1[0 + indice]       = Phiti[0];
                
                path1[1 + indice]       = Phiti[1];
                
                path1[2 + indice]       = Phiti[2];
                
                path1[3 + indice]       = rpath1[3 + id];
                
                path1[4 + indice]       = rpath1[4 + id];
                
                
                
                ind                     = (tflag11+2)*5;
                
                indice                  = (tflag11+tflag12+2)*5;
                
                
                for(j = ind  ; j < indice ; j++)
                    
                {
                    
                    path1[j] = Thits12[j - ind];
                    
                }
                
                
                path1[0 + indice]       = RX[0];
                
                path1[1 + indice]       = RX[1];
                
                path1[2 + indice]       = RX[2];
                
                path1[3 + indice]       = 0.0;
                
                path1[4 + indice]       = 0.0;
                
                
                compute_distloss(path1 , nb_pts , planes , material , fc , flp_scale , currentpath , distance , lossfac);
                
                
                if(Thits11 !=NULL)
                {
                    free(Thits11);
                    
                    Thits11 = NULL;
                }
                
                if(Thits12 !=NULL)
                {
                    free(Thits12);
                    
                    Thits12 = NULL;
                    
                }
                
                if(path1 !=NULL)
                {
                    
                    mxFree(path1);
                    
                    path1 = NULL;
                    
                }
                
            }
            
            
            // Path with 2 reflexion
            
            
            for (i = 0 ; i < npath2 ; i++)
                
            {
                
                currentpath++;
                
                id                     = i*10;
                
                
                Phitij[0]              = rpath2[0 + id];
                
                Phitij[1]              = rpath2[1 + id];
                
                Phitij[2]              = rpath2[2 + id];
                
                
                Phitj[0]               = rpath2[5 + id];
                
                Phitj[1]               = rpath2[6 + id];
                
                Phitj[2]               = rpath2[7 + id];
                
                
                Thits21                = fct_caltpts(TX, Phitij, planes , nplanes , material , &tflag21 , Phit_nt);
                
                Thits22                = fct_caltpts(Phitij, Phitj, planes , nplanes , material , &tflag22 , Phit_nt);
                
                Thits23                = fct_caltpts(Phitj, RX, planes , nplanes , material , &tflag23 , Phit_nt);
                
                
                nb_pts                 = (tflag21+tflag22+tflag23+4);
                
                path2                  = mxMalloc(nb_pts*5*sizeof(double));
                
                
                
                
                
                path2[0]               = TX[0];
                
                path2[1]               = TX[1];
                
                path2[2]               = TX[2];
                
                path2[3]               = 0.0;
                
                path2[4]               = 0.0;
                
                indice                 = (tflag21+1)*5;
                
                
                
                for(j = 5 ; j < indice  ; j++)
                    
                {
                    
                    path2[j]            = Thits21[j - 5];
                    
                }
                
                
                
                path2[0 + indice]       = Phitij[0];
                
                path2[1 + indice]       = Phitij[1];
                
                path2[2 + indice]       = Phitij[2];
                
                path2[3 + indice]       = rpath2[3 + id];
                
                path2[4 + indice]       = rpath2[4 + id];
                
                
                
                ind                     = (tflag21+2)*5;
                
                indice                  = (tflag21+tflag22 + 2)*5;
                
                
                for(j = ind ; j < indice ; j++)
                    
                {
                    
                    path2[j] = Thits22[j - ind];
                    
                }
                
                
                
                path2[0 + indice]       = Phitj[0];
                
                path2[1 + indice]       = Phitj[1];
                
                path2[2 + indice]       = Phitj[2];
                
                path2[3 + indice]       = rpath2[8 + id];
                
                path2[4 + indice]       = rpath2[9 + id];
                
                
                
                ind                     = (tflag21+tflag22+3)*5;
                
                indice                  = (tflag21+tflag22+tflag23+3)*5;
                
                
                for(j = ind ; j < indice ; j++)
                    
                {
                    
                    path2[j] = Thits23[j - ind];
                    
                }
                
                
                
                path2[0 + indice]       = RX[0];
                
                path2[1 + indice]       = RX[1];
                
                path2[2 + indice]       = RX[2];
                
                path2[3 + indice]       = 0.0;
                
                path2[4 + indice]       = 0.0;
                
                
                compute_distloss(path2 , nb_pts , planes , material , fc , flp_scale  , currentpath , distance , lossfac);
                
                
                if(Thits21 != NULL)
                {
                    free(Thits21);
                    
                    Thits21 = NULL;
                }
                
                if(Thits22 != NULL)
                {
                    free(Thits22);
                    
                    Thits22 = NULL;
                    
                }
                
                if(Thits23 != NULL)
                {
                    free(Thits23);
                    
                    Thits23 = NULL;
                    
                }
                
                
                if(path2 != NULL)
                {
                    
                    mxFree(path2);
                    
                    path2 = NULL;
                    
                }
                
                
            }
            
            
            
            // Path with 3 reflexions
            
            
            for (i = 0 ; i < npath3 ; i++)
                
            {
                
                currentpath++;
                
                id                     = i*15;
                
                
                Phitijk[0]             = rpath3[0 + id];
                
                Phitijk[1]             = rpath3[1 + id];
                
                Phitijk[2]             = rpath3[2 + id];
                
                
                Phitjk[0]              = rpath3[5 + id];
                
                Phitjk[1]              = rpath3[6 + id];
                
                Phitjk[2]              = rpath3[7 + id];
                
                
                Phitk[0]               = rpath3[10 + id];
                
                Phitk[1]               = rpath3[11 + id];
                
                Phitk[2]               = rpath3[12 + id];
                
                
                
                Thits31                = fct_caltpts(TX , Phitijk, planes , nplanes , material , &tflag31 , Phit_nt);
                
                Thits32                = fct_caltpts(Phitijk, Phitjk, planes , nplanes , material , &tflag32 , Phit_nt);
                
                Thits33                = fct_caltpts(Phitjk, Phitk, planes , nplanes , material , &tflag33 , Phit_nt);
                
                Thits34                = fct_caltpts(Phitk, RX , planes , nplanes , material , &tflag34 , Phit_nt);
                
                
                nb_pts                 = (tflag31+tflag32+tflag33+tflag34+5);
                
                path3                  = mxMalloc(nb_pts*5*sizeof(double));
                
                
                
                path3[0]               = TX[0];
                
                path3[1]               = TX[1];
                
                path3[2]               = TX[2];
                
                path3[3]               = 0.0;
                
                path3[4]               = 0.0;
                
                
                indice                 = (tflag31+1)*5;
                
                
                for(j = 5 ; j < indice ; j++)
                    
                {
                    
                    path3[j]            = Thits31[j - 5];
                    
                }
                
                
                
                path3[0 + indice]       = Phitijk[0];
                
                path3[1 + indice]       = Phitijk[1];
                
                path3[2 + indice]       = Phitijk[2];
                
                path3[3 + indice]       = rpath3[3 + id];
                
                path3[4 + indice]       = rpath3[4 + id];
                
                
                ind                     = (tflag31+2)*5;
                
                indice                  = (tflag31+tflag32 + 2)*5;
                
                
                for(j = ind; j <indice ; j++)
                    
                {
                    
                    path3[j] = Thits32[j - ind];
                    
                }
                
                
                
                path3[0 + indice]       = Phitjk[0];
                
                path3[1 + indice]       = Phitjk[1];
                
                path3[2 + indice]       = Phitjk[2];
                
                path3[3 + indice]       = rpath3[8 + id];
                
                path3[4 + indice]       = rpath3[9 + id];
                
                
                ind                     = (tflag31+tflag32+3)*5;
                
                indice                  = (tflag31+tflag32+tflag33+3)*5;
                
                
                for(j = ind; j < indice ; j++)
                    
                {
                    
                    path3[j] = Thits33[j - ind];
                    
                }
                
                
                
                path3[0 + indice]       = Phitk[0];
                
                path3[1 + indice]       = Phitk[1];
                
                path3[2 + indice]       = Phitk[2];
                
                path3[3 + indice]       = rpath3[13 + id];
                
                path3[4 + indice]       = rpath3[14 + id];
                
                ind                     = (tflag31+tflag32+tflag33+4)*5;
                
                indice                  = (tflag31+tflag32+tflag33+tflag34+4)*5;
                
                
                for(j = ind ; j < indice; j++)
                    
                {
                    
                    path3[j] = Thits34[j - ind];
                    
                }
                
                
                
                
                path3[0 + indice]       = RX[0];
                
                path3[1 + indice]       = RX[1];
                
                path3[2 + indice]       = RX[2];
                
                path3[3 + indice]       = 0.0;
                
                path3[4 + indice]       = 0.0;
                
                
                compute_distloss(path3 , nb_pts , planes , material , fc , flp_scale  , currentpath , distance , lossfac);
                
                
                if(Thits31 != NULL)
                {
                    free(Thits31);
                    
                    Thits31 = NULL;
                    
                }
                
                if(Thits32 != NULL)
                {
                    free(Thits32);
                    
                    Thits32= NULL;
                    
                }
                
                if(Thits33 != NULL)
                {
                    free(Thits33);
                    
                    Thits33 = NULL;
                    
                }
                
                if(Thits34 != NULL)
                {
                    free(Thits34);
                    
                    Thits34 = NULL;
                    
                }
                
                if(path3 != NULL)
                {
                    
                    mxFree(path3);
                    
                    path3 = NULL;
                    
                }
                
            }
            
            
            
            // Compute final Value //
            

			temp = 0.0;
            
            for (i = 0 ; i < npath ; i++)
                
            {
                
                temp += (lossfac[i]*lossfac[i]);
                
            }
            

			rs_amp[n + ld] = sqrt(temp);


			if(rpath1 !=NULL)
			{
				
				mxFree(rpath1);
				
				rpath1 = NULL;
				
			}
			
			if(rpath2 !=NULL)
			{
				
				mxFree(rpath2);
				
				rpath2 = NULL;
				
				
			}
			
			if(rpath3 !=NULL)
			{
				
				mxFree(rpath3);
				
				rpath3 = NULL;
				
				
			}
			
			
			if(nr == 0 )
			{
				
				mxFree(path1);
				
				mxFree(path2);
				
				mxFree(path3);
				
				path1 = path2 = path3 = NULL;
				
			}
			
			
			if(nr == 1 )
			{
				
				mxFree(path2);
				
				mxFree(path3);
				
				path2 = path3 = NULL;
				
				
			}
			
			if(nr == 2 )
			{
				
				mxFree(path3);
				
				path3 = NULL;
				
			}
			
			
            if(distance != NULL)
			{
				
				mxFree(distance);

				distance = NULL;
				
			}
			
            if(lossfac != NULL)
			{
				
				mxFree(lossfac);

				lossfac = NULL;
				
			}          
            
        }
    

        
    }
    
    mxFree(TX_i);
    
    mxFree(TX_j);
    
    mxFree(TX_k);
    
    mxFree(Phiti);
    
    mxFree(Phitj);
    
    mxFree(Phitij);
    
    mxFree(Phitk);
    
    mxFree(Phitjk);
    
    mxFree(Phitijk);
    
    mxFree(Phit_nt);
    
    mxFree(TX);
    
    mxFree(RX);

        
}


/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */


void compute_distloss(double *path , int num_pts , double *planes , double *material , double fc , double flp_scale  , int currentpath , double *distance , double *lossfac)

{
    
    
    int j , jd , flagP2 , ind , ind1 , ind2;
    
    double P1P2x , P1P2y , P1P2z , dj , loss_jreal , loss_jimag  , epsreal, epsimag , uy;
    
    double lossreal = 1.0, lossimag = 0.0 , tmp1 , tmp2 , tmp;
    
    double Vd , vs , vs2;
    
    double x1 , x2 , y1 , y2 , x3 , y3 , x4 , y4 , x5 , y5 ,  r1 , r2;
    
    double lambda , Ao , cte = 1.0/flp_scale;
    
    
    lambda                = c/fc;
    
    Ao                    = lambda/(4.0*PI);
    
    
    distance[currentpath] = 0.0;
    
    lossfac[currentpath]  = 1.0;
    
    for (j = 0 ; j < num_pts - 1 ; j++)
    {
        jd         = j*5;
        
        P1P2x      = path[5 + jd] - path[0 + jd];
        
        P1P2y      = path[6 + jd] - path[1 + jd];
        
        P1P2z      = path[7 + jd] - path[2 + jd];
        
        
        
        jd        += 5;
        
        flagP2     = (int)path[4 + jd];
        
        ind        = (int)path[3 + jd];
        
        ind1       = ind*22;
        
        
        uy         = planes[13 + ind1];
        
        dj         = sqrt(P1P2x*P1P2x + P1P2y*P1P2y + P1P2z*P1P2z);
        
        
        
        loss_jreal = 1.0;
        
        loss_jimag = 0.0;
        
        
        if( (flagP2 == 1))
        {
            
            ind2    = ind*6;
            
            epsreal = material[4 + ind2];
            
            epsimag = -60.0*lambda*material[5 + ind2];
            
            tmp     = 1.0/dj;
            
            Vd      = planes[12 + ind1]*P1P2x*tmp + uy*P1P2y*tmp + planes[14 + ind1]*P1P2z*tmp;
            
            vs      = fabs(Vd);
            
            vs2     = vs*vs;
            
            x1      = epsreal - (1.0 - vs2);
            
            y1      = epsimag;
            
            r1      = sqrt(x1*x1 + y1*y1);
            
            x2      = sqrt((r1 + x1)*0.5);
            
            y2      = y1/(2.0*x2);
            
            
            if(fabs(uy) > 0.5)
                
            {
                
                x3         = epsreal*vs - x2;
                
                y3         = epsimag*vs - y2;
                
                
                x4         = epsreal*vs + x2;
                
                y4         = epsimag*vs + y2;
                
                r2         = 1.0/(x4*x4 + y4*y4);
                
                loss_jreal = (x3*x4 + y3*y4)*r2;
                
                loss_jimag = (y3*x4 - x3*y4)*r2;
                
                
            }
            
            else
            {
                
                x3         = vs - x2;
                
                y3         = -y2;
                
                x4         = vs + x2;
                
                y4         = y2;
                
                r2         = 1.0/(x4*x4 + y4*y4);
                
                loss_jreal = (x3*x4 + y3*y4)*r2;
                
                loss_jimag = (y3*x4 - x3*y4)*r2;
                
            }
            
        }
        
        
        if( (flagP2 == 2))
        {
            
            ind2    = ind*6;
            
            epsreal = material[4 + ind2];
            
            epsimag = -60.0*lambda*material[5 + ind2];
            
            tmp     = 1.0/dj;
            
            Vd      = planes[12 + ind1]*P1P2x*tmp + uy*P1P2y*tmp + planes[14 + ind1]*P1P2z*tmp;
            
            vs      = fabs(Vd);
            
            vs2     = vs*vs;
            
            x1      = epsreal - (1.0 - vs2);
            
            y1      = epsimag;
            
            r1      = sqrt(x1*x1 + y1*y1);
            
            x2      = sqrt((r1 + x1)*0.5);
            
            y2      = y1/(2.0*x2);
            
            
            if(fabs(uy) > 0.5)
                
            {
                
                x3         = epsreal*vs - x2;
                
                y3         = epsimag*vs - y2;
                
                x4         = epsreal*vs + x2;
                
                y4         = epsimag*vs + y2;
                
                r2         = 1.0/(x4*x4 + y4*y4);
                
                x5         = (x3*x4 + y3*y4)*r2;
                
                y5         = (y3*x4 - x3*y4)*r2;
                
                loss_jreal = sqrt(loss_factor*(1.0 - (x5*x5 + y5*y5)));
                
            }
            
            else
            {
                
                x3         = vs - x2;
                
                y3         = -y2;
                
                x4         = vs + x2;
                
                y4         = y2;
                
                r2         = 1.0/(x4*x4 + y4*y4);
                
                x5         = (x3*x4 + y3*y4)*r2;
                
                y5         = (y3*x4 - x3*y4)*r2;
                
                loss_jreal = sqrt(loss_factor*(1.0 - (x5*x5 + y5*y5)));
                
            }
            
            
        }
        
        
        distance[currentpath] += dj;
        
        tmp1                   = (lossreal*loss_jreal - lossimag*loss_jimag);
        
        tmp2                   = (lossreal*loss_jimag + lossimag*loss_jreal);
        
        lossreal               = tmp1;
        
        lossimag               = tmp2;
        
        
    }
    
    
    lossfac[currentpath] = (Ao*sqrt(lossreal*lossreal + lossimag*lossimag))/(distance[currentpath]*cte);
    
    
}


/* ----------------------------------------------------------------------------- */


void fct_calimage(double *TX0 , double *planes , int i , double *TX_i)

{
    
    int id  = i*22;
    
    register double t_imi , ux = planes[12 + id] , uy = planes[13 + id] , uz = planes[14 + id];

	register double TX0x = TX0[0] , TX0y = TX0[1] , TX0z = TX0[2]; 
    
        
    t_imi   = -2.0*(TX0x*ux + TX0y*uy + TX0z*uz + planes[15 + id]);
    
    TX_i[0] = TX0x + ux*t_imi;
    
    TX_i[1] = TX0y + uy*t_imi;
    
    TX_i[2] = TX0z + uz*t_imi;
    
    
}


/* ----------------------------------------------------------------------------- */


bool fct_calrpt(double *P1, double *P2, double *planes , int i , double *Phit)

{
    
    int id = i*22;
    
    double  ux = planes[12 + id], uy = planes[13 + id] , uz = planes[14 + id];

	register double P1x = P1[0], P1y = P1[1] , P1z = P1[2];
  
    double ti_max, ti_maxtol , Vdi , P1P2x , P1P2y , P1P2z , Rdx , Rdy , Rdz , ti , cte;
    
    
    P1P2x     = P2[0] - P1x;
    
    P1P2y     = P2[1] - P1y;
    
    P1P2z     = P2[2] - P1z;
    
    ti_max    = sqrt(P1P2x*P1P2x + P1P2y*P1P2y + P1P2z*P1P2z);

	ti_maxtol = ti_max - t_tol;
    
    
    if (ti_max > t_tol)
    {
        
        cte     = 1.0/ti_max;
        
        Rdx     = P1P2x*cte;
        
        Rdy     = P1P2y*cte;
        
        Rdz     = P1P2z*cte;
                
        Vdi     = ux*Rdx + uy*Rdy + uz*Rdz;
        
        if (Vdi != 0.0)
        {
            
            ti = - (ux*P1x + uy*P1y + uz*P1z + planes[15 + id]) / Vdi;
            
            if ((t_tol < ti) && (ti < ti_maxtol))
            {
                
                Phit[0] = P1x + Rdx*ti;
                
                Phit[1] = P1y + Rdy*ti;
                
                Phit[2] = P1z + Rdz*ti;
                
                if ((planes[16 + id] <= Phit[0]) && (Phit[0] <= planes[17 + id]) && (planes[18 + id] <= Phit[1]) && (Phit[1]<=planes[19 + id]) && (planes[20 + id] <= Phit[2]) && (Phit[2] <= planes[21 + id]))
                    
                {
                    
                    return true;
                    
                }
                
            }
            
        }
    }
    
    return false;
    
}


/* ----------------------------------------------------------------------------- */


double * fct_caltpts(double *P1, double *P2, double *planes, int nplanes , double *material , int *nthits , double *Phit_nt)

{
    
    int  number = 0 , nt , ntd , jd  , i , ind , id;
    
    double tmax_nt, tmax_ntol;
    
    double *result=NULL;
    
    double Vd_nt , P1P2x , P1P2y , P1P2z , Rdx_nt , Rdy_nt , Rdz_nt , t_nt , tmp ;

	double P1x = P1[0], P1y = P1[1] , P1z = P1[2];
    
    double  ux_nt , uy_nt, uz_nt;
    
    double *tab_pts=NULL , *sorted_tab_pts=NULL;
    
    int *index=NULL;

	bool already_there = false;
    
    
    P1P2x     = P2[0] - P1x;
    
    P1P2y     = P2[1] - P1y;
    
    P1P2z     = P2[2] - P1z;
    
    tmax_nt   = sqrt(P1P2x*P1P2x + P1P2y*P1P2y + P1P2z*P1P2z);

	tmax_ntol = tmax_nt - t_tol;
    
    if (tmax_nt > t_tol)
        
    {
        tmp        = 1.0/tmax_nt ;
        
        Rdx_nt     = P1P2x*tmp;
        
        Rdy_nt     = P1P2y*tmp;
        
        Rdz_nt     = P1P2z*tmp;
        
        for (nt = 0 ; nt < nplanes ; nt++)
            
        {
            ntd        = nt*22;
            
            ux_nt      = planes[12 + ntd];
            
            uy_nt      = planes[13 + ntd];
            
            uz_nt      = planes[14 + ntd];
                        
            Vd_nt      = ux_nt*Rdx_nt + uy_nt*Rdy_nt + uz_nt*Rdz_nt;
            
            if (Vd_nt != 0.0)
            {
                
                t_nt = - (ux_nt*P1x + uy_nt*P1y + uz_nt*P1z + planes[15 + ntd])  / Vd_nt;

                if ((t_tol < t_nt) && (t_nt < tmax_ntol))
                {
                    
                    Phit_nt[0] = P1x + Rdx_nt*t_nt;
                    
                    Phit_nt[1] = P1y + Rdy_nt*t_nt;
                    
                    Phit_nt[2] = P1z + Rdz_nt*t_nt;
                    
                    if ((planes[16 + ntd] <= Phit_nt[0]) && (Phit_nt[0] <= planes[17 + ntd]) && (planes[18 + ntd] <= Phit_nt[1]) && (Phit_nt[1] <= planes[19 + ntd]) && ( planes[20 + ntd] <= Phit_nt[2]) && (Phit_nt[2] <= planes[21 + ntd]))
                        
                    {
						
                        if(number == 0)
                            
                        {
                            
                            jd              = number*6;
                            
                            number++;
                            
                            tab_pts         = (double *)realloc((double *)tab_pts , (jd + 6)*sizeof(double));
                            
                            
                            tab_pts[0 + jd] = Phit_nt[0];
                            
                            tab_pts[1 + jd] = Phit_nt[1];
                            
                            tab_pts[2 + jd] = Phit_nt[2];
                            
                            tab_pts[3 + jd] = nt;
                            
                            tab_pts[4 + jd] = 2.0;
                            
                            tab_pts[5 + jd] = t_nt;
                            
                        }
                        
                        else
                        {
                            
							already_there = false;
                            
                            for (i = 0 ; i < number ; i++)
                            {
                                id                = 6*i;

                                if( (fabs(tab_pts[0 + id] - Phit_nt[0]) < t_tol) && (fabs(tab_pts[1 + id] - Phit_nt[1]) < t_tol) && (fabs(tab_pts[2 + id] - Phit_nt[2]) < t_tol) )

                                {
                                    
									already_there = true;
                                    
                                }
                                
                            }
                            
                            if(already_there == false)

                            {
                                
                                jd              = number*6;
                                
                                number++;
                                
                                tab_pts         = (double *)realloc((double *)tab_pts , (jd + 6)*sizeof(double));
                                
                                tab_pts[0 + jd] = Phit_nt[0];
                                
                                tab_pts[1 + jd] = Phit_nt[1];
                                
                                tab_pts[2 + jd] = Phit_nt[2];
                                
                                tab_pts[3 + jd] = nt;
                                
                                tab_pts[4 + jd] = 2.0;
                                
                                tab_pts[5 + jd] = t_nt;
                                
                            }
                            
                        }

                        
                    }
                    
                }
                
            }
            
        }
        
        if(number > 0)
            
        {
            
            sorted_tab_pts = (double *)malloc(number*sizeof(double));
            
            index          = (int *)malloc(number*sizeof(int));
            
            for (i = 0 ; i < number ; i++)
            {
                
                index[i]          = i;
                
                sorted_tab_pts[i] = tab_pts[5 + i*6];
                
            }
            
            
            qsindex( sorted_tab_pts, index , 0 , number - 1 );
            
            
            result          = (double *)malloc(number*5*sizeof(double));
            
            for (i = 0 ; i < number ; i++)
            {
                
                ind            = 6*index[i];
                
                id             = 5*i;
                
                result[0 + id] = tab_pts[0 + ind];
                
                result[1 + id] = tab_pts[1 + ind];
                
                result[2 + id] = tab_pts[2 + ind];
                
                result[3 + id] = tab_pts[3 + ind];
                
                result[4 + id] = tab_pts[4 + ind];
                
            }
            
        }
        
    }
    
    
    free(tab_pts);
    
    free(sorted_tab_pts);
    
    free(index);
    
    nthits[0] = number;
    
    return result;
}



/* ----------------------------------------------------------------------------- */

void qsindex (double  *a, int *index , int lo, int hi)
{
//  lo is the lower index, hi is the upper index
//  of the region of array a that is to be sorted
    int i=lo, j=hi , ind;
    double x=a[(lo+hi)/2] , h;

    //  partition
    do
    {    
        while (a[i]<x) i++; 
        while (a[j]>x) j--;
        if (i<=j)
        {
            h        = a[i]; 
			a[i]     = a[j]; 
			a[j]     = h;
			ind      = index[i];
			index[i] = index[j];
			index[j] = ind;
            i++; 
			j--;
        }
    }
	while (i<=j);

    //  recursion
    if (lo<j) qsindex(a , index , lo , j);
    if (i<hi) qsindex(a , index , i , hi);
}

