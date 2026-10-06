"""Block model for the blocks-world planning problem."""


class Cube:
    """A single block of the blocks world.

    The attributes map one-to-one onto the predicates defined by the subject:

        ========  ============  ==============================================
        Subject   Attribute     Meaning
        ========  ============  ==============================================
        LIBRE     ``free``      nothing is stacked on the block and the gripper
                                is not holding it
        TENU      ``held``      the block is currently in the robot gripper
        SUR       ``on``        the block this one rests on, ``None`` when it
                                does not rest on another block
        SURTABLE  ``on_table``  the block rests directly on the table
        ========  ============  ==============================================
    """

    def __init__(self, name=None, free=None, held=None, on=None, on_table=None):
        self.name = name
        self.free = free
        self.held = held
        self.on = on
        self.on_table = on_table

    def __str__(self):
        """Return the one-line description of the block.

        The wording is the one used by the original program, so the output of
        the refactored code can be compared literally against the results
        reported in the presentation.
        """
        support = self.on.name if self.on is not None else '""'
        return "Cube {}: libre({}),tenu({}),sur({}),surtable({})".format(
            self.name, self.free, self.held, support, self.on_table
        )

    def copy_from(self, cube):
        """Copy ``cube`` into ``self``, duplicating the stack below it.

        The block referenced by ``on`` is duplicated as well, recursively, so
        that the copy shares no object with the original. Callers that need the
        copies to reference each other again must relink them afterwards; this
        is what :meth:`blocksworld.node.Node.copy_from` does.
        """
        self.name = cube.name
        self.free = cube.free
        self.held = cube.held
        if cube.on is None:
            self.on = None
        else:
            support = Cube()
            support.copy_from(cube.on)
            self.on = support
        self.on_table = cube.on_table

    def is_free(self):
        """Return the LIBRE predicate for this block."""
        return self.free

    def is_held(self):
        """Return the TENU predicate for this block."""
        return self.held

    def is_on_table(self):
        """Return the SURTABLE predicate for this block."""
        return self.on_table

    def __eq__(self, other):
        """Compare two blocks field by field, including the stack below them.

        Two blocks are equal when their name and their three boolean predicates
        match and when they rest on equal blocks. A block resting on another is
        never equal to a block resting on nothing.
        """
        if other is None:
            return False
        if self.on is not None and other.on is not None:
            return (
                self.name == other.name
                and self.free == other.free
                and self.held == other.held
                and self.on == other.on
                and self.on_table == other.on_table
            )
        if self.on is None and other.on is None:
            return (
                self.name == other.name
                and self.free == other.free
                and self.held == other.held
                and self.on_table == other.on_table
            )
        return False
