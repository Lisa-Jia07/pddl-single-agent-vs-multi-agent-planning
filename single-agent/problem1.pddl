(define (problem p1-network)
  (:domain SingleRobotRunner)

  (:objects
    g00 g01 g02 g03
    g10 g11 g12 g13
    g20 g21 g22 g23
    g30 g31 g32 g33 - gridsquare

    north east south west - direction
  )

  (:init
    ; Agent start position
    (agent-at g00)

    ; Agent initial direction
    (agent-rotated east)

    ; Other robots positions
    (robot-at g10)
    (robot-at g12)
    
    ; Obstacles
    (obstacle-at g11)

    ; Connectivity between adjacent grid cells
    ; Horizontal connections (east/west)
    (gridsquare-connected g00 g01 east)
    (gridsquare-connected g01 g00 west)
    (gridsquare-connected g01 g02 east)
    (gridsquare-connected g02 g01 west)
    (gridsquare-connected g02 g03 east)
    (gridsquare-connected g03 g02 west)
    
    (gridsquare-connected g10 g11 east)
    (gridsquare-connected g11 g10 west)
    (gridsquare-connected g11 g12 east)
    (gridsquare-connected g12 g11 west)
    (gridsquare-connected g12 g13 east)
    (gridsquare-connected g13 g12 west)
    
    (gridsquare-connected g20 g21 east)
    (gridsquare-connected g21 g20 west)
    (gridsquare-connected g21 g22 east)
    (gridsquare-connected g22 g21 west)
    (gridsquare-connected g22 g23 east)
    (gridsquare-connected g23 g22 west)
    
    (gridsquare-connected g30 g31 east)
    (gridsquare-connected g31 g30 west)
    (gridsquare-connected g31 g32 east)
    (gridsquare-connected g32 g31 west)
    (gridsquare-connected g32 g33 east)
    (gridsquare-connected g33 g32 west)
    
    ; Vertical connections (north/south)
    (gridsquare-connected g00 g10 south)
    (gridsquare-connected g10 g00 north)
    (gridsquare-connected g10 g20 south)
    (gridsquare-connected g20 g10 north)
    (gridsquare-connected g20 g30 south)
    (gridsquare-connected g30 g20 north)
    
    (gridsquare-connected g01 g11 south)
    (gridsquare-connected g11 g01 north)
    (gridsquare-connected g11 g21 south)
    (gridsquare-connected g21 g11 north)
    (gridsquare-connected g21 g31 south)
    (gridsquare-connected g31 g21 north)
    
    (gridsquare-connected g02 g12 south)
    (gridsquare-connected g12 g02 north)
    (gridsquare-connected g12 g22 south)
    (gridsquare-connected g22 g12 north)
    (gridsquare-connected g22 g32 south)
    (gridsquare-connected g32 g22 north)
    
    (gridsquare-connected g03 g13 south)
    (gridsquare-connected g13 g03 north)
    (gridsquare-connected g13 g23 south)
    (gridsquare-connected g23 g13 north)
    (gridsquare-connected g23 g33 south)
    (gridsquare-connected g33 g23 north)

    ; Rotation rules
    ; Clockwise rotation
    (rotation-adjacent north east)
    (rotation-adjacent east south)
    (rotation-adjacent south west)
    (rotation-adjacent west north)
    ; Counter-clockwise rotation
    (rotation-adjacent east north)
    (rotation-adjacent south east)
    (rotation-adjacent west south)
    (rotation-adjacent north west)
  )

  (:goal
    (and
      (agent-at g22)
    )
  )
)
