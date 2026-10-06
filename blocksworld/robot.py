"""Robot arm model and the four production rules that drive it."""


class Robot:
    """The robot arm.

    ``arm_empty`` is the BRASVIDE predicate of the subject: it is true when the
    gripper holds no block.
    """

    def __init__(self, arm_empty=None):
        self.arm_empty = arm_empty

    def __str__(self):
        """Return the one-line description of the arm, as in the original program."""
        return "Robot: brasvide({})".format(self.arm_empty)

    def copy_from(self, robot):
        """Copy ``robot`` into ``self``."""
        self.arm_empty = robot.arm_empty

    def arm_is_empty(self):
        """Return the BRASVIDE predicate."""
        return self.arm_empty

    def __eq__(self, other):
        """Two arms are equal when they are both empty or both loaded."""
        return self.arm_empty == other.arm_empty

    def hold(self, cube):
        """Pick ``cube`` up and update the state, implementing TENIR.

        Two production rules of the subject are covered:

        * **R1** - the arm is empty, the block is free and it lies on the table.
        * **R2** - the arm is empty, the block is free and it lies on another
          block, which becomes free in turn.

        Returns ``True`` when a rule fired and the state was updated, ``False``
        when no precondition was satisfied and the state is left untouched.
        """
        if self.arm_is_empty() and cube.is_free() and cube.is_on_table():
            # R1: the block was resting on the table.
            self.arm_empty = False
            cube.free = False
            cube.held = True
            cube.on = None
            cube.on_table = False
            return True
        if self.arm_is_empty() and cube.is_free() and cube.on is not None:
            # R2: the block was resting on another block, which is freed.
            self.arm_empty = False
            cube.free = False
            cube.held = True
            cube.on.free = True
            cube.on = None
            cube.on_table = False
            return True
        return False

    def put(self, cube, target):
        """Put ``cube`` down on ``target`` and update the state, implementing POSER.

        ``target`` is the block to stack ``cube`` onto, or ``None`` to put it
        back on the table. Two production rules of the subject are covered:

        * **R4** - the arm holds the block and ``target`` is free, so the block
          is stacked onto it and ``target`` stops being free.
        * **R3** - the arm holds the block, which is put back on the table.

        Note that the two rules are tested in that order and that R3 has no
        guard on ``target``: asking to stack onto a block that is *not* free
        falls through to R3 and puts the block on the table instead of failing.
        This is the behaviour of the submitted program and it is kept
        deliberately, because A* relies on the exact set of children each state
        produces. The resulting child duplicates the "put on the table"
        operator and is discarded by the duplicate detection in
        :func:`blocksworld.astar.astar`.

        Returns ``True`` when a rule fired, ``False`` otherwise.
        """
        if target is not None:
            if not self.arm_is_empty() and cube.is_held() and target.is_free():
                # R4: stack the held block onto another block.
                self.arm_empty = True
                cube.free = True
                cube.held = False
                cube.on = target
                target.free = False
                cube.on_table = False
                return True

        if not self.arm_is_empty() and cube.is_held():
            # R3: put the held block back on the table.
            self.arm_empty = True
            cube.free = True
            cube.held = False
            cube.on = None
            cube.on_table = True
            return True

        return False
