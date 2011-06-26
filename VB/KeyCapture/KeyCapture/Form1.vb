Public Class Form1

    'normal keys
    Private Sub Form1_KeyDown(ByVal sender As Object, ByVal e As System.Windows.Forms.KeyEventArgs) Handles Me.KeyDown
        MessageBox.Show("KeyDown: " + "keycode: " + e.KeyCode.ToString + " keyvalue: " + e.KeyValue.ToString + " keydata: " + e.KeyData.ToString)
    End Sub


    Public Const VK_SNAPSHOT As Integer = &H2C 'PrintScreen key
    Public Const WM_HOTKEY As Integer = &H312

    Public Declare Function RegisterHotKey Lib "user32" (ByVal hwnd As IntPtr, ByVal id As Integer, ByVal fsModifiers As Integer, ByVal vk As Integer) As Integer
    Public Declare Function UnregisterHotKey Lib "user32" (ByVal hwnd As IntPtr, ByVal id As Integer) As Integer

    Private Sub Form1_Load(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles MyBase.Load
        ' To register single key, set fsModifiers parameter as 0
        Call RegisterHotKey(Me.Handle, 9, 0, VK_SNAPSHOT)
    End Sub

    Protected Overrides Sub WndProc(ByRef m As System.Windows.Forms.Message)
        If m.Msg = WM_HOTKEY Then
            MessageBox.Show("PrintScreen was pressed.")
            Me.Show()
        End If
        MyBase.WndProc(m)
    End Sub

    Private Sub Form1_FormClosing(ByVal sender As System.Object, ByVal e As System.Windows.Forms.FormClosingEventArgs) Handles MyBase.FormClosing
        Call UnregisterHotKey(Me.Handle, 9)
    End Sub

End Class
