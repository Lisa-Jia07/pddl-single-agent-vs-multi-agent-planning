(define (domain MultiRobotRunner)
(:requirements
    :typing
    :negative-preconditions
    :action-costs
)

(:types
    robot gridsquare direction
)

(:predicates
    ;Which robot at which location
    (robot-at ?r - robot ?loc - gridsquare)
    ;Which robot faces which direction
    (robot-facing ?r - robot ?d - direction)
    
    ;Obstacle at which location
    (obstacle-at ?loc - gridsquare)
    
    ;Grid square adjacent in particular direction
    (gridsquare-connected ?from ?to - gridsquare ?d - direction)
    ;Rotation adjacency
    (rotation-adjacent ?from ?to - direction)
    
    ;Which robot chooses action
    (turn ?r - robot)
    ;Which robot chooses action after a robot chooses action
    (next-turn ?r1 ?r2 - robot)
    (first-robot ?r - robot)
    (last-robot ?r - robot)
    
    ;Two phases (select action / perform action)
    (phase-select)
    (phase-apply)
    
    ;What the robot plan to do in this round (moveforward/rotate/wait)
    (planned-moveforward ?r - robot ?from ?to - gridsquare)
    (planned-rotate ?r - robot ?from ?to - direction)
    (planned-wait ?r - robot ?loc - gridsquare)
    
    ;Prevent Vertex Collision
    ;A location that the robot planned to stay for the next round
    (reserved-location ?loc - gridsquare)
    ;Prevent Edge (Swapping) Collision
    (reserved-edge ?from ?to - gridsquare)
    
    ; Mark robot has already selected action in this round
    (select-action-done ?r - robot)
    
    ; Mark the Goal location for a specific robot
    (goal-location ?r - robot ?loc - gridsquare)
)

(:functions
    (total-cost)
)

; ---------------------------
; SELECTION PHASE
; robots decide actions here
; ---------------------------

; choose moveforward (not last robot)
; only allowed if no obstacle and no collision conflict
(:action select-moveforward
    :parameters (?r - robot ?from ?to - gridsquare ?d - direction ?nxt - robot)
    :precondition (and
        (phase-select)
        (turn ?r)
        (not (last-robot ?r))
        (next-turn ?r ?nxt)
        (not (select-action-done ?r))

        (robot-at ?r ?from)
        (not (goal-location ?r ?from))
        (robot-facing ?r ?d)
        (gridsquare-connected ?from ?to ?d)
        (not (obstacle-at ?to))

        ; Avoid Vertex Collision
        (not (reserved-location ?to))

        ; Avoid Edge (Swapping) Collision
        (not (reserved-edge ?to ?from))
    )
    :effect (and
        (planned-moveforward ?r ?from ?to)
        (reserved-location ?to)
        (reserved-edge ?from ?to)
        (select-action-done ?r)
        (not (turn ?r))
        (turn ?nxt)
    )
 )

; choose rotate (not last robot)
; rotation stays in same cell, so also needs reservation
(:action select-rotate
    :parameters (?r - robot ?loc - gridsquare ?fromdir ?todir - direction ?nxt - robot)
    :precondition (and 
        (phase-select)
        (turn ?r)
        (not (last-robot ?r))
        (next-turn ?r ?nxt)
        (not (select-action-done ?r))
        
        (robot-at ?r ?loc)
        (not (goal-location ?r ?loc))
        (robot-facing ?r ?fromdir)
        (rotation-adjacent ?fromdir ?todir)
        
        (not (reserved-location ?loc))   
    )
    :effect (and 
        (planned-rotate ?r ?fromdir ?todir)
        (reserved-location ?loc)
        (select-action-done ?r)
        (not (turn ?r))
        (turn ?nxt)
    )
)

; choose wait (not last robot)
(:action select-wait
    :parameters (?r - robot ?loc - gridsquare ?nxt - robot)
    :precondition (and
        (phase-select)
        (turn ?r)
        (not (last-robot ?r))
        (next-turn ?r ?nxt)
        (not (select-action-done ?r))
        
        (robot-at ?r ?loc)
        
        (not (reserved-location ?loc))
    )
    :effect (and
        (planned-wait ?r ?loc)
        (reserved-location ?loc)
        (select-action-done ?r)
        (not (turn ?r))
        (turn ?nxt)
    )
)

; Last robot selects moveforward
; After the last robot selects, the domain switches
; from selection phase to apply phase
(:action select-moveforward-last-robot
    :parameters (?r - robot ?from ?to - gridsquare ?d - direction ?first - robot)
    :precondition (and
        (phase-select)
        (turn ?r)
        (last-robot ?r)
        (first-robot ?first)
        (not (select-action-done ?r))

        (robot-at ?r ?from)
        (not (goal-location ?r ?from))
        (robot-facing ?r ?d)
        (gridsquare-connected ?from ?to ?d)
        (not (obstacle-at ?to))

        (not (reserved-location ?to))
        (not (reserved-edge ?to ?from))
    )
    :effect (and
        (planned-moveforward ?r ?from ?to)
        (reserved-location ?to)
        (reserved-edge ?from ?to)
        (select-action-done ?r)

        (not (turn ?r))
        (not (phase-select))
        (phase-apply)
        (turn ?first)
    )
 )

; Last robot selects rotate
 (:action select-rotate-last-robot
    :parameters (?r - robot ?loc - gridsquare ?fromdir ?todir - direction ?first - robot)
    :precondition (and
        (phase-select)
        (turn ?r)
        (last-robot ?r)
        (first-robot ?first)
        (not (select-action-done ?r))

        (robot-at ?r ?loc)
        (not (goal-location ?r ?loc))
        (robot-facing ?r ?fromdir)
        (rotation-adjacent ?fromdir ?todir)

        (not (reserved-location ?loc))
    )
    :effect (and
        (planned-rotate ?r ?fromdir ?todir)
        (reserved-location ?loc)
        (select-action-done ?r)

        (not (turn ?r))
        (not (phase-select))
        (phase-apply)
        (turn ?first)
    )
)

; Last robot selects wait
(:action select-wait-last-robot
    :parameters (?r - robot ?loc - gridsquare ?first - robot)
    :precondition (and
        (phase-select)
        (turn ?r)
        (last-robot ?r)
        (first-robot ?first)
        (not (select-action-done ?r))

        (robot-at ?r ?loc)
        (not (reserved-location ?loc))
    )
    :effect (and
        (planned-wait ?r ?loc)
        (reserved-location ?loc)
        (select-action-done ?r)

        (not (turn ?r))
        (not (phase-select))
        (phase-apply)
        (turn ?first)
    )
)

; ---------------------------
; APPLY PHASE
; execute chosen actions
; ---------------------------

; apply moveforward
(:action apply-moveforward
    :parameters (?r - robot ?from ?to - gridsquare ?nxt - robot)
    :precondition (and
        (phase-apply)
        (turn ?r)
        (not (last-robot ?r))
        (next-turn ?r ?nxt)
    
        (planned-moveforward ?r ?from ?to)
        (robot-at ?r ?from)
        (reserved-location ?to)
        (reserved-edge ?from ?to)
        (select-action-done ?r)
    )
    :effect (and
        (not (robot-at ?r ?from))
        (robot-at ?r ?to)
    
        (not (planned-moveforward ?r ?from ?to))
        (not (reserved-location ?to))
        (not (reserved-edge ?from ?to))
        (not (select-action-done ?r))
    
        (not (turn ?r))
        (turn ?nxt)
        (increase (total-cost) 1)
    )
)
    
; apply rotate
(:action apply-rotate
    :parameters (?r - robot ?loc - gridsquare ?fromd ?tod - direction ?nxt - robot)
    :precondition (and
        (phase-apply)
        (turn ?r)
        (not (last-robot ?r))
        (next-turn ?r ?nxt)
    
        (planned-rotate ?r ?fromd ?tod)
        (robot-at ?r ?loc)
        (robot-facing ?r ?fromd)
        (reserved-location ?loc)
        (select-action-done ?r)
    )
    :effect (and
        (not (robot-facing ?r ?fromd))
        (robot-facing ?r ?tod)
    
        (not (planned-rotate ?r ?fromd ?tod))
        (not (reserved-location ?loc))
        (not (select-action-done ?r))
    
        (not (turn ?r))
        (turn ?nxt)
        (increase (total-cost) 2)
    )
)
    
; apply wait
(:action apply-wait
    :parameters (?r - robot ?loc - gridsquare ?nxt - robot)
    :precondition (and
        (phase-apply)
        (turn ?r)
        (not (last-robot ?r))
        (next-turn ?r ?nxt)
    
        (planned-wait ?r ?loc)
        (robot-at ?r ?loc)
        (reserved-location ?loc)
        (select-action-done ?r)
    )
    :effect (and
        (not (planned-wait ?r ?loc))
        (not (reserved-location ?loc))
        (not (select-action-done ?r))
    
        (not (turn ?r))
        (turn ?nxt)
        (increase (total-cost) 0)
    )
)

; APPLY PHASE - last robot
; after last robot applies, go back to SELECT phase
(:action apply-moveforward-last-robot
    :parameters (?r - robot ?from ?to - gridsquare ?first - robot)
    :precondition (and
        (phase-apply)
        (turn ?r)
        (last-robot ?r)
        (first-robot ?first)

        (planned-moveforward ?r ?from ?to)
        (robot-at ?r ?from)
        (reserved-location ?to)
        (reserved-edge ?from ?to)
        (select-action-done ?r)
    )
    :effect (and
        (not (robot-at ?r ?from))
        (robot-at ?r ?to)

        (not (planned-moveforward ?r ?from ?to))
        (not (reserved-location ?to))
        (not (reserved-edge ?from ?to))
        (not (select-action-done ?r))

        (not (turn ?r))
        (not (phase-apply))
        (phase-select)
        (turn ?first)
        (increase (total-cost) 1)
    )
)

(:action apply-rotate-last-robot
    :parameters (?r - robot ?loc - gridsquare ?fromd ?tod - direction ?first - robot)
    :precondition (and
        (phase-apply)
        (turn ?r)
        (last-robot ?r)
        (first-robot ?first)

        (planned-rotate ?r ?fromd ?tod)
        (robot-at ?r ?loc)
        (robot-facing ?r ?fromd)
        (reserved-location ?loc)
        (select-action-done ?r)
    )
    :effect (and
        (not (robot-facing ?r ?fromd))
        (robot-facing ?r ?tod)
    
        (not (planned-rotate ?r ?fromd ?tod))
        (not (reserved-location ?loc))
        (not (select-action-done ?r))

        (not (turn ?r))
        (not (phase-apply))
        (phase-select)
        (turn ?first)
        (increase (total-cost) 2)
    )
)

(:action apply-wait-last-robot
    :parameters (?r - robot ?loc - gridsquare ?first - robot)
    :precondition (and
        (phase-apply)
        (turn ?r)
        (last-robot ?r)
        (first-robot ?first)

        (planned-wait ?r ?loc)
        (robot-at ?r ?loc)
        (reserved-location ?loc)
        (select-action-done ?r)
    )   
    :effect (and
        (not (planned-wait ?r ?loc))
        (not (reserved-location ?loc))
        (not (select-action-done ?r))

        (not (turn ?r))
        (not (phase-apply))
        (phase-select)
        (turn ?first)
        (increase (total-cost) 0)
    )
)
)
