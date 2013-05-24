@echo off
%3
cd %1

rem set path=c:\env\imagemagick\6.3.0\VisualMagick\bin;%path%

rem convert images\pinhole-faugeras.bmp images\pinhole-faugeras.eps
rem convert images\epipolar-faugeras.bmp images\epipolar-faugeras.eps
rem convert images\pinhole-far.bmp images\pinhole-far.eps
rem convert images\pinhole-near.bmp images\pinhole-near.eps
rem convert images\perspect.bmp images\perspect.eps
rem convert images\env-top.bmp images\env-top.eps
rem convert images\env.bmp images\env.eps
rem convert boards\checker.emf images\checker.eps
rem convert images\restriction.bmp images\restriction.eps
rem convert images\lens-dist.bmp images\lens-dist.eps
rem convert images\lens-dist-rad.bmp images\lens-dist-rad.eps
rem convert images\epipolar.bmp images\epipolar.eps
rem convert images\square-ref.bmp images\square-ref.eps
rem convert images\gap-ref.bmp images\gap-ref.eps
rem convert images\hough-params.bmp images\hough-params.eps
rem convert images\sectors.bmp images\sectors.eps
rem convert images\points.png images\points.eps
rem convert images\tatuzinho.bmp images\tatuzinho.eps
rem convert images\videotrack.gif images\videotrack.eps
rem convert images\trace-l_x-all.bmp images\trace-l_x-all.eps
rem convert images\trace-l_y-all.bmp images\trace-l_y-all.eps
rem convert images\trace-l_x-fit.bmp images\trace-l_x-fit.eps
rem convert images\trace-l_y-fit.bmp images\trace-l_y-fit.eps
rem convert images\trace-l_x-filt.bmp images\trace-l_x-filt.eps
rem convert images\trace-l_y-filt.bmp images\trace-l_y-filt.eps
rem convert images\rats-colour.jpg images\rats-colour.eps
rem convert images\plane_xy.bmp images\plane_xy.eps
rem convert images\plane_xz.bmp images\plane_xz.eps
rem convert images\plane_yz.bmp images\plane_yz.eps
rem convert images\plane_xy-l.bmp images\plane_xy-l.eps
rem convert images\plane_xy-r.bmp images\plane_xy-r.eps

rem convert images\result-model-scenario.bmp 		images\result-model-scenario.eps
rem convert images\result-model-acceleration.bmp	images\result-model-acceleration.eps
rem convert images\result-model-occupacy.bmp 		images\result-model-occupacy.eps
rem convert images\result-model-projection.bmp 	images\result-model-projection.eps
rem convert images\result-model-velocity.bmp 		images\result-model-velocity.eps

rem convert images\result-simulated-velocity.bmp 		images\result-simulated-velocity.eps
rem convert images\result-simulated-acceleration.bmp 	images\result-simulated-acceleration.eps

rem convert images\result-real-scenario.bmp 		images\result-real-scenario.eps
rem convert images\result-real-acceleration.bmp 	images\result-real-acceleration.eps
rem convert images\result-real-occupacy.bmp 		images\result-real-occupacy.eps
rem convert images\result-real-projection.bmp 	images\result-real-projection.eps
rem convert images\result-real-velocity.bmp 		images\result-real-velocity.eps

rem convert images\result-real-zones-alt.bmp 		images\result-real-zones.eps

rem convert images\restriction_left_side.bmp images\restrictions-left-side.eps
rem convert images\restriction_right_side.bmp images\restrictions-right-side.eps
rem convert images\restriction_right_side_flip.bmp images\restrictions-right-side-flip.eps
rem convert images/result-simulated-scenario.bmp images/result-simulated-scenario.eps

rem convert images\result-simulated-occupacy.bmp 		images\result-simulated-occupacy.eps
rem convert images\result-simulated-projection.bmp 	images\result-simulated-projection.eps 	

makeindex < %2.idx > %2.ind
bibtex %2
latex --src-specials %2.tex
dvips %2.dvi
