Public Class Class1

    Private mstrLine As String

    Property Line() As String
        Get
            Return mstrLine
        End Get
        Set(ByVal Value As String)
            mstrLine = Value
        End Set
    End Property

    Public Sub New(ByVal Value As String)
        mstrLine = Value
    End Sub

End Class
