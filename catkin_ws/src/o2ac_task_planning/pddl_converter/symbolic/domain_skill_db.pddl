(define (domain skill-database)
  (:requirements :adl)
  (:types
    assembly - object  ; The type "assembly" includes individual parts (= atomic assemblies)
    robot - robot
  )

  (:predicates 
    ; General object predicates
    (available ?x - object) ; True if object is in tray or tool in holder

    ; Assembly/part properties
    (complete ?a - assembly)
    (part-of ?part ?whole - assembly)  ; True if part belongs into the whole

    ; Part states
    (incorporated ?part ?whole - assembly)  ; "Incorporated" parts are currently part of the assembly (they are "assembled", or "mounted")
    (object-is-stable ?o - assembly)  ; This is true if a screw has been fastened, or the object is horizontal (e.g. bearing, peg...) (this might not work for all parts)
    (object-is-placed ?o - assembly)  ; True when the object is at the goal/target location and not in the tray
    (handover-allowed ?o - assembly)  ; True when the object can be passed to another through handover (small objects cannot)

    ; Assembly hierarchy
    (assemble-order ?part1 ?part2 ?whole - assembly)  ; Part 2 needs to come after Part 1

    ; Mating properties
    (requires-screws ?a - assembly) ; True if the part needs to be fastened with screws
    (requires-insertion ?a - assembly) ; True if the part needs to be inserted
    (requires-reorientation ?a - assembly) ; True if the part needs to be reoriented

    ; Robot & tool properties
    (robot-carries-an-object ?r - robot) ; This could probably be evaluated with a (for all) clause on robot-carries-this-object instead, but having this is probably easier.
    (robot-carries-this-object ?r - robot ?o - object)  ; Can be a tool or part
  )


  ; === Actions for pick/place/release    

  (:action pick
    :parameters (?robot - robot ?part - object)
    :precondition(
      and
        (available ?part)
        (not (robot-carries-an-object ?robot))
    )
    :effect(
      and
        (robot-carries-an-object ?robot)
        (robot-carries-this-object ?robot ?part)
        (not (available ?part))  ; This implies that only one of each part exists in the scene
    )
  )

  (:action pick-orient-dual-arm
    :parameters (?robot - robot ?part - object ?helper_robot - robot)
    :precondition(
      and
        (available ?part)
        (not (robot-carries-an-object ?robot))
        (not (robot-carries-an-object ?helper_robot))
        (requires-reorientation ?part)
        (handover-allowed ?part)
        (not(= ?robot ?helper_robot))
    )
    :effect(
      and
        (robot-carries-an-object ?robot)
        (robot-carries-this-object ?robot ?part)
        (not (available ?part))  ; This implies that only one of each part exists in the scene
        (not (requires-reorientation ?part))
    )
  )

  (:action orient
    :parameters (?robot - robot ?part - object)
    :precondition(
      and
        (robot-carries-this-object ?robot ?part)
        (requires-reorientation ?part)
    )
    :effect(
      and
        (robot-carries-an-object ?robot)
        (robot-carries-this-object ?robot ?part)
        (not (requires-reorientation ?part))
    )
  )

  (:action place
    :parameters (?robot - robot ?part ?prev ?whole - assembly)
    :precondition(
      and
        (robot-carries-this-object ?robot ?part)
        (part-of ?part ?whole)
        (assemble-order ?prev ?part ?whole)
        (incorporated ?prev ?whole)
        (not (requires-reorientation ?part))
        (not (requires-insertion ?part))
    )
    :effect(
      and
        (object-is-placed ?part)
        (when (not (requires-screws ?part))
          (object-is-stable ?part)
        )
    )
  )
    
  (:action release
    :parameters (?robot - robot ?part ?whole - assembly)
    :precondition(
      and
        (robot-carries-this-object ?robot ?part)
        (object-is-stable ?part)
        (part-of ?part ?whole)
        (forall (?prev - assembly) ; = all previous parts have been assembled
          (imply (assemble-order ?prev ?part ?whole)
            (incorporated ?prev ?whole)
          )
        )
    )
    :effect(
      and 
        (not (robot-carries-this-object ?robot ?part))
        (not (robot-carries-an-object ?robot))
        (when (not (requires-screws ?part)) (incorporated ?part ?whole))
    )
  )

  ; === Tool-related actions (equip/pick-screw/fasten)
 
  (:action fasten-with-screw
    :parameters (?robot - robot ?part - assembly ?whole - assembly)
    :precondition(
      and
        (not (robot-carries-an-object ?robot))
        (not (requires-insertion ?part))
        (object-is-placed ?part)
        (requires-screws ?part)
        (part-of ?part ?whole)
    )
    :effect(
      and
        (object-is-stable ?part)
        (incorporated ?part ?whole)
    )
  )

  (:action fasten-with-screw-dual-arm
    :parameters (?robot - robot ?part - assembly ?whole - assembly ?helper_robot - robot)
    :precondition(
      and
        (not (robot-carries-an-object ?robot))
        (not (requires-insertion ?part))
        (robot-carries-this-object ?helper_robot ?part) ; robot holding the part to be screwed
        (object-is-placed ?part)
        (requires-screws ?part)
        (part-of ?part ?whole)
        (not(= ?robot ?helper_robot))
    )
    :effect(
      and
        (object-is-stable ?part)
        (incorporated ?part ?whole)
        (not (robot-carries-this-object ?helper_robot ?part))
        (not (robot-carries-an-object ?helper_robot))
    )
  )

  (:action insert-dual-arm
    :parameters (?robot - robot ?part - assembly ?helper_robot - robot ?part2 - assembly ?whole - assembly)
    :precondition(
      and
        (robot-carries-this-object ?robot ?part)
        (requires-insertion ?part)
        (robot-carries-this-object ?helper_robot ?part2)
        (not (requires-reorientation ?part))
        (part-of ?part ?whole)
        (part-of ?part2 ?whole)
        (not(= ?robot ?helper_robot))
        (not(= ?part ?part2))
        (forall (?p - assembly) 
          (imply (part-of ?p ?whole) 
          (and
            (not (requires-reorientation ?p)) ; Make sure all the parts are oriented before insertion
            (complete ?p)
          )
          )
        )
    )
    :effect(
      and
        (not (requires-insertion ?part))
        (not (requires-insertion ?part2))
        (object-is-stable ?part)
        (object-is-stable ?part2)
        (incorporated ?part ?whole)
        (incorporated ?part2 ?whole)
        (not (robot-carries-this-object ?robot ?part))
        (not (robot-carries-an-object ?robot))
        (not (robot-carries-this-object ?helper_robot ?part2))
        (robot-carries-this-object ?helper_robot ?whole)
        (not (available ?whole))
    )
  )

  (:action insert
    :parameters (?robot - robot ?part - assembly ?part2 - assembly ?whole - assembly)
    :precondition(
      and
        (robot-carries-this-object ?robot ?part)
        (requires-insertion ?part)
        (not (requires-reorientation ?part))
        (part-of ?part ?whole)
        (part-of ?part2 ?whole)
        (not(= ?part ?part2))
        (forall (?p - assembly) 
          (imply (part-of ?p ?whole) 
          (and
            (not (requires-reorientation ?p)) ; Make sure all the parts are oriented before insertion
            (complete ?p)
          )
          )
        )
    )
    :effect(
      and
        (not (requires-insertion ?part))
        (object-is-stable ?part)
        (when (not (requires-screws ?part)) (incorporated ?part ?whole))
        (object-is-placed ?part)
    )
  )

  ; Ending action (can only be accessed if all part of the assembly are incorporated and released)

  (:action finished
    :parameters (?whole - assembly)
    :precondition(
      and
        (not (complete ?whole))
        (forall (?p - assembly ?r - robot)
          (imply (part-of ?p ?whole)
            (and
                (incorporated ?p ?whole)
                (not (robot-carries-this-object ?r ?p))
            )
          )
        )
    )
    :effect(
      and
        (complete ?whole)
        (available ?whole)
    )
  )
)
